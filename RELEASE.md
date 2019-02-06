ADMerlin Releases
==================

The latest untagged master branch can be obtained at
https://github.com/areaDetector/ADMerlin.

Tagged source code releases from R4-0 onward can be obtained at 
https://github.com/areaDetector/ADMerlin/releases.


Release Notes
=============

R4-1 (XXX-Feb-2019)
---
* Comply with v2.0 of protocol for Uom devices - still backward compatible with Merlin Quad protocol
* Remove redundant protocol versions
* Change Medipix to Merlin. Merlin is the detector that uses Medipix chips.
  The Merlin protocol was copied by Uom to control their BPMs which do not use Medipix chips.
* Changed configure/RELEASE files for compatibility with areaDetector R3-3.
* Added merlinApp/op/Makefile to autoconvert adl files to opi, ui, edl.

v4.0 (19-Sept-2016)
----
* First 'non-DLS' version of Merlin based on ADCore 2-4. Thanks to Mark Rivers and Matthew Moore for the work in converting to ADCore and removal of DLS specifics.

earlier
------------------
Earlier release tags are internal to Diamond Light Source 

