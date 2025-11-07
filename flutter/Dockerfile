FROM archlinux:latest

RUN pacman -Syu --noconfirm \
    curl \
    git \
    unzip \
    xz \
    zip \
    glu \
    fzf \
    zoxide \
    which \
    xorg-server \
    xorg-xauth \
    xorg-xhost \
    libx11 \
    libxext \
    libxrender \
    libxrandr \
    libxi \
    libxcursor \
    libxinerama \
    mesa \
    gtk3 \
    cmake \
    ninja \
    clang \
    pkgconf \
    at-spi2-core

# ---- Install Flutter ----
ENV FLUTTER_HOME=/opt/flutter
RUN git clone https://github.com/flutter/flutter.git -b stable $FLUTTER_HOME
ENV PATH="$PATH:$FLUTTER_HOME/bin"

# ---- Pre-download tools ----
RUN flutter doctor

COPY ./flutter /flutter

# ---- Set workdir ----
WORKDIR /flutter

# ---- Default command ----
CMD ["bash"]
