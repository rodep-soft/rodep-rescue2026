FROM archlinux:latest

# ---- Set Japanese mirrors and update ----
RUN pacman -Syu --noconfirm pacman-contrib \
 && sed -i '1iServer = https://ftp.jaist.ac.jp/pub/Linux/ArchLinux/$repo/os/$arch' /etc/pacman.d/mirrorlist \
 && for i in 1 2 3 4 5; do pacman -Syu --noconfirm \
    curl git unzip xz fish zip glu fzf zoxide which vim \
    xorg-server xorg-xauth xorg-xhost libx11 libxext libxrender \
    libxrandr libxi libxcursor libxinerama mesa gtk3 cmake ninja \
    clang pkgconf at-spi2-core && break || sleep 5; done

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
