/****************************************************************************
 *
 * fkie_potree_rviz_plugin
 * Copyright © 2018-2023 Fraunhofer FKIE
 * Author: Timo Röhling
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#include "cloud_meta_data.h"

#include <json/reader.h>

#include <fstream>

#define JSON_GET_EX(Type, Var, Data, Key, Error) \
    do                                           \
    {                                            \
        if (Data[Key].isNull())                  \
            throw std::runtime_error(Error);     \
        Var = Data[Key].as##Type();              \
    } while (0)

#define JSON_GET(Type, Var, Data, Key) \
    JSON_GET_EX(Type, Var, Data, Key, "missing " #Type " key: " Key)

namespace fkie_potree_rviz_plugin
{

namespace
{

CloudMetaData::PointAttribute pointAttributeFromString(const std::string& attr)
{
    if (attr == "POSITION_CARTESIAN")
        return {attr, 12, 3, 4, CloudMetaData::PointAttribute::Int};
    if (attr == "COLOR_PACKED" || attr == "RGBA_PACKED")
        return {"COLOR_PACKED", 4, 4, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "RGB_PACKED")
        return {attr, 3, 3, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "INTENSITY")
        return {attr, 2, 1, 2, CloudMetaData::PointAttribute::UInt};
    if (attr == "CLASSIFICATION")
        return {attr, 1, 1, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "RETURN_NUMBER" || attr == "NUMBER_OF_RETURNS")
        return {attr, 1, 1, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "SOURCE_ID")
        return {attr, 2, 1, 2, CloudMetaData::PointAttribute::UInt};
    if (attr == "GPS_TIME")
        return {attr, 8, 1, 8, CloudMetaData::PointAttribute::Float};
    if (attr == "NORMAL_SPHEREMAPPED")
        return {attr, 2, 2, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "NORMAL_OCT16")
        return {attr, 2, 2, 1, CloudMetaData::PointAttribute::UInt};
    if (attr == "NORMAL" || attr == "NORMAL_FLOATS")
        return {attr, 12, 3, 4, CloudMetaData::PointAttribute::Float};
    if (attr == "SPACING")
        return {attr, 4, 1, 4, CloudMetaData::PointAttribute::Float};
    if (attr == "INDICES")
        return {attr, 4, 1, 4, CloudMetaData::PointAttribute::UInt};
    throw std::runtime_error("unsupported point attribute " + attr);
}

CloudMetaData::PointAttribute parsePointAttribute(const Json::Value& attr)
{
    std::string name, type_str;
    std::size_t size, num_elems, elem_size;
    JSON_GET(String, name, attr, "name");
    JSON_GET(String, type_str, attr, "type");
    JSON_GET(UInt, size, attr, "size");
    if (!attr["numElements"].isNull())
        JSON_GET(UInt, num_elems, attr, "numElements");
    else
        JSON_GET(UInt, num_elems, attr, "elements");
    JSON_GET(UInt, elem_size, attr, "elementSize");
    CloudMetaData::PointAttribute::Type type =
        CloudMetaData::PointAttribute::None;
    if (type_str.substr(0, 3) == "int")
        type = CloudMetaData::PointAttribute::Int;
    if (type_str.substr(0, 4) == "uint")
        type = CloudMetaData::PointAttribute::UInt;
    if (type_str.substr(0, 5) == "float")
        type = CloudMetaData::PointAttribute::Float;
    return {name, size, num_elems, elem_size, type};
}

}  // namespace

CloudMetaData::CloudMetaData(const fs::path& file_name)
{
    std::ifstream f(file_name.c_str());
    if (!f.good())
        throw std::runtime_error(std::string("cannot open file: ")
                                 + file_name.string());
    Json::Reader reader;
    Json::Value data;
    if (!reader.parse(f, data, false))
    {
        throw std::runtime_error(std::string("cannot parse meta data: ")
                                 + reader.getFormattedErrorMessages());
    }
    std::string version;
    JSON_GET(String, version, data, "version");
    if (version.substr(0, 2) == "1.")
    {
        version_ = POTREE_1_X;
        parsePotree1(data);
    }
    else if (version.substr(0, 2) == "2.")
    {
        version_ = POTREE_2_X;
        parsePotree2(data);
    }
    if (point_byte_size_ == 0)
        throw std::runtime_error("unsupported Potree " + version + " format");
    cloud_path_ = file_name.parent_path();
}

void CloudMetaData::parsePotree1(Json::Value& data)
{
    JSON_GET(String, octree_dir_, data, "octreeDir");
    JSON_GET(UInt, point_count_, data, "points");
    JSON_GET(UInt, hierarchy_step_size_, data, "hierarchyStepSize");
    JSON_GET(Float, spacing_, data, "spacing");
    JSON_GET(Float, scale_[0], data, "scale");
    scale_[1] = scale_[2] = scale_[0];
    Json::Value bb = data["boundingBox"];
    if (bb.isNull())
        throw std::runtime_error("missing bounding box");
    float lx, ly, lz, ux, uy, uz;
    JSON_GET(Float, lx, bb, "lx");
    JSON_GET(Float, ly, bb, "ly");
    JSON_GET(Float, lz, bb, "lz");
    JSON_GET(Float, ux, bb, "ux");
    JSON_GET(Float, uy, bb, "uy");
    JSON_GET(Float, uz, bb, "uz");
    bounding_box_.setExtents(lx, ly, lz, ux, uy, uz);
    Json::Value attrs = data["pointAttributes"];
    if (attrs.isNull())
        throw std::runtime_error("missing point attributes");
    point_attributes_.clear();
    point_byte_size_ = 0;
    for (Json::ArrayIndex i = 0; i < attrs.size(); ++i)
    {
        if (attrs[i].isString())
        {
            std::string val;
            JSON_GET_EX(String, val, attrs, i,
                        "invalid point attribute array entry");
            PointAttribute point_attr = pointAttributeFromString(val);
            point_attributes_.push_back(point_attr);
            point_byte_size_ += point_attr.size;
        }
        else
        {
            PointAttribute point_attr = parsePointAttribute(attrs[i]);
            point_attributes_.push_back(point_attr);
            point_byte_size_ += point_attr.size;
        }
    }
}

void CloudMetaData::parsePotree2(Json::Value& data)
{
    JSON_GET(UInt, point_count_, data, "points");
    JSON_GET(Float, spacing_, data, "spacing");
    Json::Value scale = data["scale"];
    if (scale.isNull())
        throw std::runtime_error("missing scale");
    JSON_GET_EX(Float, scale_[0], scale, 0, "invalid scale array entry");
    JSON_GET_EX(Float, scale_[1], scale, 1, "invalid scale array entry");
    JSON_GET_EX(Float, scale_[2], scale, 2, "invalid scale array entry");
    Json::Value offset = data["offset"];
    if (offset.isNull())
        throw std::runtime_error("missing offset");
    JSON_GET_EX(Float, offset_[0], offset, 0, "invalid offset array entry");
    JSON_GET_EX(Float, offset_[1], offset, 1, "invalid offset array entry");
    JSON_GET_EX(Float, offset_[2], offset, 2, "invalid offset array entry");
    Json::Value bb = data["boundingBox"];
    if (bb.isNull())
        throw std::runtime_error("missing bounding box");
    Json::Value bb_min = bb["min"];
    if (bb_min.isNull())
        throw std::runtime_error("missing bounding box min element");
    Json::Value bb_max = bb["max"];
    if (bb_max.isNull())
        throw std::runtime_error("missing bounding box min element");
    float lx, ly, lz, ux, uy, uz;
    JSON_GET_EX(Float, lx, bb_min, 0, "invalid bounding box min array entry");
    JSON_GET_EX(Float, ly, bb_min, 1, "invalid bounding box min array entry");
    JSON_GET_EX(Float, lz, bb_min, 2, "invalid bounding box min array entry");
    JSON_GET_EX(Float, ux, bb_max, 0, "invalid bounding box max array entry");
    JSON_GET_EX(Float, uy, bb_max, 1, "invalid bounding box max array entry");
    JSON_GET_EX(Float, uz, bb_max, 2, "invalid bounding box max array entry");
    bounding_box_.setExtents(lx, ly, lz, ux, uy, uz);
    Json::Value hierarchy = data["hierarchy"];
    if (hierarchy.isNull())
        throw std::runtime_error("missing hierarchy");
    JSON_GET(UInt, hierarchy_root_size_, hierarchy, "firstChunkSize");
    JSON_GET(UInt, hierarchy_step_size_, hierarchy, "stepSize");
    Json::Value attrs = data["attributes"];
    if (attrs.isNull())
        throw std::runtime_error("missing point attributes");
    point_attributes_.clear();
    point_byte_size_ = 0;
    for (Json::ArrayIndex i = 0; i < attrs.size(); ++i)
    {
        PointAttribute point_attr = parsePointAttribute(attrs[i]);
        point_attributes_.push_back(point_attr);
        point_byte_size_ += point_attr.size;
    }
}

}  // namespace fkie_potree_rviz_plugin
