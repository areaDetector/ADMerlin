#!/bin/bash

export PATH=$PATH:/local/DPbin/epics/epics_2015-03-31/extensions/bin/linux-x86_64/:/APSshare/caqtdm/bin/

#!export CAQTDM_DISPLAY_PATH=$CAQTDM_DISPLAY_PATH:/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADCore-R2-1/ADApp/op/ui/:/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADMedipix/medipixAPP/opi/ui/:/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADMedipix/medipixApp/opi/ui
# Kevin changed this on 2015-02-20
export CAQTDM_DISPLAY_PATH=/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADCore-R2-1/ADApp/op/ui/:/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADMedipix/medipixAPP/opi/ui/:/local/DPbin/epics/epics_2015-03-31/synApps_5_8/support/areaDetector-2-1/ADMedipix/medipixApp/opi/ui

caQtDM -x -macro "P=dp_merlin_xrd46:, R=cam1:" ADMerlinQuad.ui &

