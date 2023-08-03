# Use the official Ubuntu 20.04 LTS image as the base image
FROM ubuntu:latest

# Dependencies
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       apt-transport-https \
       ca-certificates \
       curl \
       gnupg-agent \
       software-properties-common \
       docker.io \
       python3-pip \
       sudo \
       git \
       python3 \
       openssh-client


# Add Docker's official GPG key
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Add Docker's official repository
RUN echo \
  "deb [arch=amd64 signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
RUN apt-get update \
    && apt-get install -y --no-install-recommends docker-ce docker-ce-cli containerd.io

# Install pandas
RUN python3 -m pip install pandas \
    matplotlib

# Install sudo
RUN apt-get update && apt-get install -y sudo

# Adding new user
RUN useradd -m -s /bin/bash andreeabruchental \
    && echo "andreeabruchental ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER andreeabruchental

# Expose the Docker daemon port
EXPOSE 2375

CMD ["dockerd", "--host=unix:///var/run/docker.sock", "--host=tcp://0.0.0.0:2375"]
