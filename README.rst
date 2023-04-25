FKIE Potree rviz plugin
=======================

|rviz|

About
-----

`Potree <https://github.com/potree/potree>`_ is a free open-source
WebGL based point cloud renderer for large point clouds. This plugin
enables `rviz <https://github.com/ros-visualization/rviz>`_ to render
the Potree point clouds.

Preprocessing
-------------

The plugin requires some preprocessing to render point clouds.
The `Potree Converter <https://github.com/potree/PotreeConverter>`_ will
create an appropriate multi-resolution point cloud from LAS, LAZ, PLY,
XYZ or PTX files.

The plugin currently supports uncompressed RGB point clouds created
with converter versions 1.6 up to 2.1.

Acknowledgements
----------------

The original rendering algorithm was developed by Markus Schütz
as part of his
`thesis <https://www.cg.tuwien.ac.at/research/publications/2016/SCHUETZ-2016-POT/SCHUETZ-2016-POT-thesis.pdf>`_.

This plugin has been inspired by a similar
`plugin for Unity <https://github.com/SFraissTU/BA_PointCloud>`_,
written by Simon Fraiss.

.. |rviz| image:: https://raw.githubusercontent.com/fkie/potree_rviz_plugin/master/screenshot.png

