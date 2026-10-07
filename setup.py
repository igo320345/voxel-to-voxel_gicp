# -*- coding: utf-8 -*-
import os
import sys
import glob
import shutil
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


PLAT_TO_CMAKE = {
    "win32": "Win32",
    "win-amd64": "x64",
    "win-arm32": "ARM",
    "win-arm64": "ARM64",
}


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        super().__init__(name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        ext_fullpath = self.get_ext_fullpath(ext.name)
        extdir = os.path.abspath(os.path.dirname(ext_fullpath))

        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = "Release"
        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DEXAMPLE_VERSION_INFO={self.distribution.get_version()}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DBUILD_PYTHON_BINDINGS=ON",
            "-DBUILD_apps=OFF",
        ]

        build_args = []

        # ------------------------------------------------------------
        # macOS OpenMP handling
        #
        # Prefer pixi/conda prefix if available.
        # Fallback to Homebrew only outside pixi/conda.
        # ------------------------------------------------------------
        if sys.platform == "darwin":
            conda_prefix = (
                os.environ.get("CONDA_PREFIX")
                or os.environ.get("PIXI_ENVIRONMENT_PATH")
                or os.environ.get("PREFIX")
            )

            if conda_prefix and os.path.exists(
                os.path.join(conda_prefix, "lib", "libomp.dylib")
            ):
                omp_prefix = conda_prefix
            else:
                try:
                    omp_prefix = subprocess.check_output(
                        ["brew", "--prefix", "libomp"],
                        text=True,
                    ).strip()
                except Exception:
                    omp_prefix = "/opt/homebrew/opt/libomp"

            omp_lib = os.path.join(omp_prefix, "lib", "libomp.dylib")
            omp_include = os.path.join(omp_prefix, "include")

            cmake_args += [
                f"-DOpenMP_ROOT={omp_prefix}",
                f"-DCMAKE_PREFIX_PATH={omp_prefix}",
                f"-DOpenMP_C_FLAGS=-Xpreprocessor -fopenmp",
                f"-DOpenMP_CXX_FLAGS=-Xpreprocessor -fopenmp",
                f"-DOpenMP_C_LIB_NAMES=omp",
                f"-DOpenMP_CXX_LIB_NAMES=omp",
                f"-DOpenMP_omp_LIBRARY={omp_lib}",
                f"-DOpenMP_C_INCLUDE_DIR={omp_include}",
                f"-DOpenMP_CXX_INCLUDE_DIR={omp_include}",

                # Important: make dylibs inside the package find each other.
                f"-DCMAKE_INSTALL_RPATH={extdir}",
                "-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON",
                "-DCMAKE_MACOSX_RPATH=ON",
            ]

            # Also add extension directory and pixi lib to runtime rpath.
            cmake_args += [
                f"-DCMAKE_BUILD_RPATH={extdir};{os.path.join(omp_prefix, 'lib')}",
            ]

        if self.compiler.compiler_type == "msvc":
            single_config = any(x in cmake_generator for x in {"NMake", "Ninja"})
            contains_arch = any(x in cmake_generator for x in {"ARM", "Win64"})

            if not single_config and not contains_arch:
                cmake_args += ["-A", PLAT_TO_CMAKE[self.plat_name]]

            if not single_config:
                cmake_args += [
                    f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{cfg.upper()}={extdir}"
                ]
                build_args += ["--config", cfg]

        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ:
            if hasattr(self, "parallel") and self.parallel:
                build_args += [f"-j{self.parallel}"]

        os.makedirs(self.build_temp, exist_ok=True)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args,
            cwd=self.build_temp,
        )

        subprocess.check_call(
            ["cmake", "--build", "."] + build_args,
            cwd=self.build_temp,
        )

        # ------------------------------------------------------------
        # macOS: copy libfast_gicp.dylib next to pygicp*.so
        # ------------------------------------------------------------
        if sys.platform == "darwin":
            dylib_name = "libfast_gicp.dylib"

            candidates = glob.glob(
                os.path.join(self.build_temp, "**", dylib_name),
                recursive=True,
            )

            candidates = [
                p for p in candidates
                if os.path.isfile(p)
                and os.path.realpath(p).startswith(os.path.realpath(self.build_temp))
            ]

            if candidates:
                dst = os.path.join(extdir, dylib_name)
                shutil.copy2(candidates[0], dst)

                # Make pygicp*.so find libfast_gicp.dylib next to itself.
                try:
                    subprocess.check_call(
                        ["install_name_tool", "-add_rpath", "@loader_path", ext_fullpath]
                    )
                except subprocess.CalledProcessError:
                    pass

                # Make libfast_gicp.dylib find OpenMP from pixi/conda prefix.
                if sys.platform == "darwin":
                    try:
                        subprocess.check_call(
                            [
                                "install_name_tool",
                                "-add_rpath",
                                os.path.join(omp_prefix, "lib"),
                                dst,
                            ]
                        )
                    except subprocess.CalledProcessError:
                        pass

            else:
                print(
                    f"WARNING: Could not find {dylib_name} in build output. "
                    "If import fails, copy libfast_gicp.dylib next to pygicp*.so."
                )


setup(
    name="pygicp",
    version="0.1.0",
    author="igo320341",
    author_email="tkachev@itmo.ru",
    description="Voxel-to-Voxel Generalized ICP for Fast and Accurate LiDAR Odometry",
    long_description="",
    ext_modules=[CMakeExtension("pygicp")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)