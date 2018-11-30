/****************************************************************************
 *
 * fkie_potree_rviz_plugin
 * Copyright © 2018 Fraunhofer FKIE
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
    do { \
        if (Data[Key].isNull()) \
            throw std::runtime_error(Error); \
        Var = Data[Key].as ## Type(); \
    } while(0)

#define JSON_GET(Type, Var, Data, Key) \
    JSON_GET_EX(Type, Var, Data, Key, "missing " #Type " key: " Key)

namespace fkie_potree_rviz_plugin
{

void CloudMetaData::readFromJson(const fs::path& file_name)
{
    std::ifstream f(file_name.c_str());
    if (!f.good()) throw std::runtime_error(std::string("cannot open file: ") + file_name.string());
    Json::Reader reader;
    Json::Value data;
    if (!reader.parse(f, data, false))
    {
        throw std::runtime_error(std::string("cannot parse meta data: ") + reader.getFormattedErrorMessages());
    }
    JSON_GET(String, octree_dir_, data, "octreeDir");
    JSON_GET(UInt, point_count_, data, "points");
    JSON_GET(UInt, hierarchy_step_size_, data, "hierarchyStepSize");
    JSON_GET(Float, spacing_, data, "spacing");
    JSON_GET(Float, scale_, data, "scale");
    Json::Value bb = data["boundingBox"];
    if (bb.isNull()) throw std::runtime_error("missing bounding box");
    float lx, ly, lz, ux, uy, uz;
    JSON_GET(Float, lx, bb, "lx");
    JSON_GET(Float, ly, bb, "ly");
    JSON_GET(Float, lz, bb, "lz");
    JSON_GET(Float, ux, bb, "ux");
    JSON_GET(Float, uy, bb, "uy");
    JSON_GET(Float, uz, bb, "uz");
    bounding_box_.setExtents(lx, ly, lz, ux, uy, uz);
    Json::Value attr = data["pointAttributes"];
    if (attr.isNull()) throw std::runtime_error("missing point attributes");
    point_attributes_.clear();
    point_byte_size_ = 0;
    for (Json::ArrayIndex i = 0; i < attr.size(); ++i)
    {
        std::string val;
        JSON_GET_EX(String, val, attr, i, "invalid point attribute array entry");
        std::size_t sz = sizeOf(val);
        if (sz == 0) throw std::runtime_error("unsupported point attribute: " + val);
        point_attributes_.push_back(val);
        point_byte_size_ += sz;
    }
    cloud_path_ = file_name.parent_path();
}

std::size_t CloudMetaData::sizeOf(const std::string& attr)
{
    if (attr == "POSITION_CARTESIAN") return 12;
    if (attr == "COLOR_PACKED") return 4;
    if (attr == "INTENSITY") return 2;
    if (attr == "CLASSIFICATION") return 1;
    if (attr == "RETURN_NUMBER") return 1;
    if (attr == "NUMBER_OF_RETURNS") return 1;
    if (attr == "SOURCE_ID") return 2;
    if (attr == "GPS_TIME") return 8;
    if (attr == "NORMAL_SPHEREMAPPED") return 2;
    if (attr == "NORMAL_OCT16") return 2;
    if (attr == "NORMAL") return 12;
    return 0;
}

} // namespace fkie_rviz_plugin_potree
