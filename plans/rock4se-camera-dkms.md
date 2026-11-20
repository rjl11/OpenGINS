# OV9281 on the ROCK 4SE: surviving kernel upgrades and boots

## Context
The OV9281 camera for the optical flow ([optical-flow-aiding.md](optical-flow-aiding.md); INNO-MAKER CAM-MIPI9281RAW-V2, manual in `external/CAM-OV9281RAW-V2/`) runs on a Radxa ROCK 4SE (RK3399, DietPi / Armbian kernel 6.18) since 2026-11-19. `pi/flow/README.md`, "Setup on a Radxa ROCK 4SE", has the details. It needs two drivers built outside DietPi's kernel:
- **`rkisp1`**: Armbian's rockchip64 kernels don't enable `CONFIG_VIDEO_ROCKCHIP_ISP1` (checked for the 6.18 `current` and 7.3 `edge` builds).
- **A patched `ov9282`**: it reports the sensor's mono Y8/Y10 output as RGGB, because `rkisp1` takes no mono input.

`pi/flow/rock4se/setup.sh install` builds both by hand into `/lib/modules/<ver>/updates/`. Capture goes through V4L2 (`flow --v4l2`), because libcamera 0.4 has no ov9281 helper for `rkisp1`.

Two things still break:
- **Kernel upgrades:** a DietPi kernel upgrade leaves the camera without drivers until `setup.sh install` is run again.
- **Boots:** after every boot, `setup.sh pipeline` must be run by hand before `flow --v4l2` works.

**Why not a custom kernel.** Compiling a kernel with the driver built in was considered (2026-11-20) and not chosen:
- It would remove only the `rkisp1` workaround. The `ov9282` patch and the V4L2 capture stay either way.
- It costs hours of building on the board, or a Linux build setup elsewhere, plus holding back or rebuilding every kernel update.
- Kernel options don't speed up the flow, which spends its time in OpenCV in user space.
- The speed difference measured on 2026-11-19 is a runtime setting: flow took 5.9 ms per frame at 100 fps and 1.75 ms at 300 fps, because the CPU governor runs the cores slowly under light load.

So this plan keeps DietPi's kernel and adds DKMS for the two modules and a boot service for the pipeline and the CPU tuning.

## Approach

### 1. One module build script, shared by `setup.sh` and DKMS
New `pi/flow/rock4se/build_modules.sh KVER OUTDIR`: the download, patch and `make` part of today's `setup.sh install`, unchanged in substance.
- Fetches `rkisp1` and `ov9282.c` from kernel.org stable at the tag `v<KVER base>`, so the source always matches the kernel being built for.
- Applies the `sed` that turns `Y8/Y10` into `SRGGB8/SRGGB10`.
- Builds against `/lib/modules/KVER/build`.

### 2. DKMS package `opengins-ov9281`
`pi/flow/rock4se/dkms/dkms.conf`, installed to `/usr/src/opengins-ov9281-1/` together with `build_modules.sh`:
- `MAKE[0]="sh build_modules.sh ${kernelver} ."`
- `BUILT_MODULE_NAME[0]=rockchip-isp1` in `rkisp1/`, `BUILT_MODULE_NAME[1]=ov9282` in `ov9282/`, both with `DEST_MODULE_LOCATION=/updates`.
- `AUTOINSTALL=yes`, so `/etc/kernel/postinst.d/dkms` rebuilds both when a new kernel and its `linux-headers-current-rockchip64` arrive.
- Limitation: kernel.org must be reachable at upgrade time. If it isn't, the build fails visibly in `dkms status`, and a later `dkms autoinstall` retries.

`setup.sh install` changes:
- Installs `dkms`, copies the package, runs `dkms add` and `dkms install`.
- Removes the hand-copied `.ko` files from `updates/`.
- Overlay, `user_overlays` and the `video` group stay as they are.

### 3. Boot service `opengins-camera.service`
`pi/flow/rock4se/opengins-camera.service` (oneshot, `RemainAfterExit`, after `systemd-udev-settle`), installed by a new `setup.sh service`. It runs `setup.sh boot`, which:
- Waits up to 10 s for the `rkisp1` media device.
- Runs the existing `pipeline` step.
- Sets the `performance` governor on the A72 cluster (`/sys/devices/system/cpu/cpufreq/policy4`) only. The A53s keep `schedutil`.

To make it run as root, `setup.sh` uses a `SUDO` variable that is empty when the script already runs as root.

Pinning the flow to the A72s stays a documented `taskset -c 4,5` on the `flow` command line, with no code change.

### 4. Docs
- `pi/flow/README.md`, ROCK 4SE section: DKMS, the service, the governor, and the new measurements.
- The private build-host notes: modules now under DKMS and the pipeline set at boot, so drop "run `setup.sh pipeline` after every boot".

## Files
- New: `pi/flow/rock4se/build_modules.sh`, `pi/flow/rock4se/dkms/dkms.conf`, `pi/flow/rock4se/opengins-camera.service`.
- Changed: `pi/flow/rock4se/setup.sh` (install through DKMS; `service` and `boot` commands; the `SUDO` variable), `pi/flow/README.md`, the private build-host notes.

## Verification (on the ROCK 4SE)
1. `setup.sh install`: `dkms status` shows `opengins-ov9281/1, 6.18.60-current-rockchip64: installed`, and `modinfo -n ov9282` points into `updates/dkms`.
2. **Another kernel:** install `linux-headers-edge-rockchip64` (7.3) and run `sudo dkms build opengins-ov9281/1 -k 7.3.1-edge-rockchip64`. This checks that the build follows another kernel version without booting it. Remove those headers again afterwards.
3. **Reboot with no manual steps:** `systemctl status opengins-camera` is active, `setup.sh status` shows the SRGGB8 640×400 pipeline at 100 fps, and `pi/flow/build/flow --dry-run --v4l2` runs straight away.
4. **Governor effect:** run flow at 100 fps for 20 s with and without `taskset -c 4,5`. Record the time per frame next to the earlier 5.9 ms, and check the temperature (`/sys/class/thermal/thermal_zone0/temp`) stays reasonable over a few minutes.

## Later
A pull request to Armbian enabling `CONFIG_VIDEO_ROCKCHIP_ISP1=m` in its rockchip64 kernel config would let the stock kernel carry `rkisp1`, leaving only the `ov9282` patch. Mono input support in `rkisp1` upstream would remove that too.
