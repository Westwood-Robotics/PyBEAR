#!usr/bin/env python

################################################################################
# Copyright 2020 Westwood Robotics Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

__author__ = "Westwood Robotics Corporation"
__email__ = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2020 Westwood Robotics"
__date__ = "Jan. 01, 2020"

__version__ = "0.1.2"
__status__ = "Production"

TIMEOUT_DEFAULT = 0.001  # In seconds
BULK_TIMEOUT_DEFAULT = 0.001  # In seconds
PING_TIMEOUT = 2*TIMEOUT_DEFAULT
BULK_COMM_RETRIES = 3  # set this number to change how many times PyBEAR will try to resend the packet

#
# TIMEOUT_MAX = 0.000125 # In seconds
