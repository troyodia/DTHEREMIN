FROM ubuntu:24.04

#install necessary packages
RUN DEBAIN_FRONTEND=noninteractive \
    apt-get update \
    && apt-get install -y wget bzip2 make unzip cppcheck clang-format git

# create a non-root user named "stm32user"
# put in root group as Github actions need permissions
# to create tmp files
RUN useradd -rm -d /home/stm32 -s /bin/bash -g root -G sudo \
    -u 1001 stm32user

USER stm32user
WORKDIR /home/stm32