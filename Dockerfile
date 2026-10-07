FROM ubuntu:24.04 AS hpm-toolchain

# Install the tested HPMicro binary toolchain used by hpm_board.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    binutils ca-certificates curl file \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

ARG HPM_TOOLCHAIN_URL=https://github.com/hpmicro/riscv-gnu-toolchain/releases/download/2023.10.18/rv32imac_zicsr_zifencei_multilib_b_ext-linux.tar.gz
RUN curl -fL --retry 5 --retry-all-errors \
        "${HPM_TOOLCHAIN_URL}" -o /tmp/hpm-riscv-toolchain.tar.gz \
    && tar -xzf /tmp/hpm-riscv-toolchain.tar.gz -C /opt \
    && mv /opt/rv32imac_zicsr_zifencei_multilib_b_ext-linux /opt/riscv32-none-elf \
    && rm /tmp/hpm-riscv-toolchain.tar.gz \
    && find /opt/riscv32-none-elf -type f -exec sh -c \
        'if file "$1" | grep -q "ELF 64-bit.*x86-64"; then strip "$1"; fi' _ {} \;

# Keep this check in a separate layer so a failed ABI check does not discard the
# expensive toolchain build cache.
RUN printf 'int main(void) { return 0; }\n' \
        | /opt/riscv32-none-elf/bin/riscv32-unknown-elf-gcc \
            -march=rv32imac -mabi=ilp32 -specs=nano.specs -x c - \
            -o /tmp/rv32imac-ilp32-smoke.elf \
    && test -s /tmp/rv32imac-ilp32-smoke.elf \
    && rm /tmp/rv32imac-ilp32-smoke.elf

FROM ubuntu:24.04 AS ci

ARG TARGETARCH
ARG TARGETARCH_UNAME=x86_64

# Set bash as the default shell
SHELL ["/bin/bash", "-c"]

# Configure timezone and locale
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV TZ=Etc/UTC
ENV DEBIAN_FRONTEND=noninteractive

# Install system tools, libraries, and compilers
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
    # General utilities
    tzdata \
    vim wget curl \
    gnupg2 ca-certificates \
    zsh usbutils \
    cmake make ninja-build \
    git sudo \
    zip unzip xz-utils \
    openssh-client \
    dfu-util \
    # Host toolchain
    libc6-dev gcc-14 g++-14 \
    pkg-config libusb-1.0-0-dev \
    libgtest-dev \
    # Firmware dependencies (HPM SDK)
    libmpc3 \
    python3 python3-pip python3-venv \
    python3-yaml python3-jinja2 \
    # Cleanup
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    # Configure GCC 14 as the default compiler
    && dpkg-divert --divert /usr/bin/gcc.distrib --rename /usr/bin/gcc \
    && dpkg-divert --divert /usr/bin/g++.distrib --rename /usr/bin/g++ \
    && dpkg-divert --divert /usr/bin/cc.distrib --rename /usr/bin/cc \
    && dpkg-divert --divert /usr/bin/c++.distrib --rename /usr/bin/c++ \
    && dpkg-divert --divert /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc.distrib --rename /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc \
    && dpkg-divert --divert /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++.distrib --rename /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++ \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-14 50 \
    && update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-14 50 \
    && update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 50 \
    && update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 50 \
    && ln -sf /usr/bin/gcc-14 /usr/bin/${TARGETARCH_UNAME}-linux-gnu-gcc \
    && ln -sf /usr/bin/g++-14 /usr/bin/${TARGETARCH_UNAME}-linux-gnu-g++

COPY --from=hpm-toolchain /opt/riscv32-none-elf /opt/riscv32-none-elf
ENV GNURISCV_TOOLCHAIN_PATH=/opt/riscv32-none-elf
ENV PATH="${GNURISCV_TOOLCHAIN_PATH}/bin:${PATH}"

# Download and install ARM GNU Toolchain
RUN test "${TARGETARCH}" = "amd64" \
    && VERSION=15.3.rel1 \
    && wget -q https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/${VERSION}/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi.tar.xz \
        -O arm-gnu-toolchain.tar.xz \
    && tar -xf arm-gnu-toolchain.tar.xz -C /opt/ \
    && rm arm-gnu-toolchain.tar.xz \
    && mv /opt/arm-gnu-toolchain-${VERSION}-${TARGETARCH_UNAME}-arm-none-eabi /opt/arm-none-eabi
ENV GNUARM_TOOLCHAIN_PATH=/opt/arm-none-eabi
ENV PATH="${GNUARM_TOOLCHAIN_PATH}/bin:${PATH}"

# Install the same LLVM major version used on developer machines directly from
# the Ubuntu 24.04 repositories.
RUN LLVM_VERSION=20 \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    clangd-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} clang-format-${LLVM_VERSION} \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    && update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-${LLVM_VERSION} 200 \
    && update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${LLVM_VERSION} 200 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 200 \
    && clangd --version | grep -q "version ${LLVM_VERSION}\." \
    && clang-tidy --version | grep -q "version ${LLVM_VERSION}\." \
    && clang-format --version | grep -q "version ${LLVM_VERSION}\."

FROM ci AS develop

# Install Node.js 24 LTS (required by Codex CLI)
RUN curl -fsSL https://deb.nodesource.com/setup_24.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/*

# Add optional Tsinghua mirror configuration for CN users
COPY <<EOF /etc/apt/sources.list.d/ubuntu.sources.cn.bak
Types: deb
URIs: http://mirrors.tuna.tsinghua.edu.cn/ubuntu/
Suites: noble noble-updates noble-security
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

# Configure 'ubuntu' user and sudo privileges
RUN chsh -s /bin/zsh ubuntu && \
    echo "ubuntu ALL=(ALL:ALL) NOPASSWD:ALL" >> /etc/sudoers

# Precreate generic XDG-style parent directories for direct bind mounts under ubuntu's home.
RUN mkdir -p \
        /home/ubuntu/.agents \
        /home/ubuntu/.cache \
        /home/ubuntu/.config \
        /home/ubuntu/.local/share \
        /home/ubuntu/.local/state && \
    chown -R ubuntu:ubuntu /home/ubuntu/.agents /home/ubuntu/.cache /home/ubuntu/.config /home/ubuntu/.local

WORKDIR /home/ubuntu
ENV USER=ubuntu
ENV WORKDIR=/home/ubuntu
USER ubuntu

# Install Oh My Zsh and configure theme
RUN sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)" \
    && sed -i 's/ZSH_THEME=\"[a-z0-9\\-]*\"/ZSH_THEME="af-magic"/g' ~/.zshrc
