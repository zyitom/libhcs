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

FROM ubuntu:24.04 AS wch-toolchain

ARG TARGETARCH
ARG WCH_TOOLCHAIN_GCC_VERSION=15.2.0
ARG WCH_TOOLCHAIN_RESOURCE_ID=2030114123741700098
ARG WCH_TOOLCHAIN_ARCHIVE=MRS_Toolchain_Linux_X64_V240.tar.xz
ARG WCH_TOOLCHAIN_ARCHIVE_SIZE=411269512
ARG WCH_TOOLCHAIN_SHA256=1fae593d27e24466f17c2df0fd00f746143f587fe33e912a78e35142fef82a6d

# MounRiver publishes this X64 package through a short-lived signed URL. Pin
# the immutable resource ID and archive digest, then extract only the GCC15
# compiler needed by ch32_board; OpenOCD and GUI debuggers stay on the host.
RUN test "${TARGETARCH}" = "amd64" \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
    ca-certificates curl jq xz-utils \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/* /tmp/* \
    && download_url="$(curl -fsSL --retry 5 --retry-all-errors \
        "https://api.mounriver.com/mountriver/api/version/fetchRecentOpenOcdUrl?resourceId=${WCH_TOOLCHAIN_RESOURCE_ID}" \
        | jq -er '.result | select(type == "string" and length > 0)')" \
    && case "${download_url}" in \
        *"/${WCH_TOOLCHAIN_ARCHIVE}?"*) ;; \
        *) echo "Unexpected MounRiver download URL" >&2; exit 1 ;; \
    esac \
    && curl -fL --retry 5 --retry-all-errors \
        "${download_url}" -o "/tmp/${WCH_TOOLCHAIN_ARCHIVE}" \
    && test "$(stat -c '%s' "/tmp/${WCH_TOOLCHAIN_ARCHIVE}")" = \
        "${WCH_TOOLCHAIN_ARCHIVE_SIZE}" \
    && echo "${WCH_TOOLCHAIN_SHA256}  /tmp/${WCH_TOOLCHAIN_ARCHIVE}" \
        | sha256sum -c - \
    && mkdir -p /opt/wch-gcc15 \
    && tar -xJf "/tmp/${WCH_TOOLCHAIN_ARCHIVE}" -C /opt/wch-gcc15 \
        --strip-components=2 "Toolchain/RISC-V Embedded GCC15" \
    && rm "/tmp/${WCH_TOOLCHAIN_ARCHIVE}" \
    && test "$(/opt/wch-gcc15/bin/riscv32-wch-elf-gcc -dumpfullversion)" = \
        "${WCH_TOOLCHAIN_GCC_VERSION}"

# Verify the exact ABI required by the CH32H417 build in a separate layer so a
# failed smoke test does not discard the downloaded toolchain cache.
RUN printf 'int main(void) { return 0; }\n' \
        | /opt/wch-gcc15/bin/riscv32-wch-elf-gcc \
            -march=rv32imafc_zicsr_zifencei -mabi=ilp32f \
            -specs=nano.specs -x c - -o /tmp/wch-ilp32f-smoke.elf \
    && /opt/wch-gcc15/bin/riscv32-wch-elf-readelf \
        -h /tmp/wch-ilp32f-smoke.elf | grep -q 'single-float ABI' \
    && rm /tmp/wch-ilp32f-smoke.elf

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

# Keep the HPM and WCH RISC-V toolchains independent. Their compiler prefixes
# and CMake selection variables are intentionally different.
COPY --from=hpm-toolchain /opt/riscv32-none-elf /opt/riscv32-none-elf
COPY --from=wch-toolchain /opt/wch-gcc15 /opt/wch-gcc15
ENV GNURISCV_TOOLCHAIN_PATH=/opt/riscv32-none-elf
ENV WCH_TOOLCHAIN_PATH=/opt/wch-gcc15
ENV WCH_TOOLCHAIN_PREFIX=riscv32-wch-elf-
ENV PATH="${WCH_TOOLCHAIN_PATH}/bin:${GNURISCV_TOOLCHAIN_PATH}/bin:${PATH}"

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
