#  Copyright (C) 2018-2020 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# CARMA Docker Configuration Script
#
# Performs all necessary tasks related to generation of a generic CARMA docker image
# suitable for deployment to a vehicle. The generic image will still need to be 
# configured for each vehicle by means of volume mapping configuration files and
# networking mapping at run time
#
# Build Arguments:
# SSH_PRIVATE_KEY - If the extra package repository to be used during the build requires
#                   authentication, please pass in the necessary SSH private key in text
#                   form here, most likely via "$(cat ~/.ssh/id_rsa)". This data is not 
#                   present in the final output image. Default = none
#
# EXTRA_PACKAGES - The repo to checkout any additional packages from at build time. 
#                  Default = none


# /////////////////////////////////////////////////////////////////////////////
# Stage 1 - Acquire the CARMA source as well as any extra packages
# /////////////////////////////////////////////////////////////////////////////

FROM usdotfhwastolcandidate/autoware.ai:zephyr AS base-image

FROM base-image AS source-code

RUN mkdir ~/src
COPY --chown=carma . /home/carma/src/carma-platform/
RUN ~/src/carma-platform/docker/checkout.bash

# /////////////////////////////////////////////////////////////////////////////
# Stage 2 - Build and install the software 
# /////////////////////////////////////////////////////////////////////////////


FROM base-image AS install

# Copy the source files from the previous stage and build/install
RUN mkdir ~/carma_ws
COPY --from=source-code --chown=carma /home/carma/src /home/carma/carma_ws/src


RUN ~/carma_ws/src/carma-platform/docker/install.sh

# /////////////////////////////////////////////////////////////////////////////
# Stage 3 - Finalize deployment
# /////////////////////////////////////////////////////////////////////////////


FROM base-image

ARG BUILD_DATE="NULL"
ARG VCS_REF="NULL"
ARG VERSION="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="CARMA"
LABEL org.label-schema.description="Binary application for the CARMA Platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/usdot-fhwa-stol/carma-platform"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Migrate the files from the install stage
COPY --from=install --chown=carma /opt/carma /opt/carma
COPY --from=install --chown=carma /root/.bashrc /home/carma/.bashrc

CMD "roslaunch carma carma_docker.launch"
