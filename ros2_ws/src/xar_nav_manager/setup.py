#!/usr/bin/env python3
from glob import glob
from setuptools import setup, find_packages

pkg = "xar_nav_manager"

setup(
    name=pkg,                   # ROS package name  ⟵ keep as-is
    version="0.1.0",
    packages=find_packages(),   # finds xar_nav_manager/*
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Voice-driven waypoint sequencer",
    license="Apache-2.0",

    # ---------- installed data ------------------------------------------------
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{pkg}"]),                             # marker
        (f"share/{pkg}/launch", glob("launch/*.py")),      # launch files
        (f"share/{pkg}", ["package.xml"]),                 # manifest
    ],

    # ---------- entry-points: one wrapper per node ---------------------------
    entry_points={
        "console_scripts": [
            # module path must match the *Python* package name
            "nav_manager       = xar_nav_manager.nav_manager:main",
            "spin_driver       = xar_nav_manager.spin_driver:main",
            "battery_publisher = xar_nav_manager.battery_publisher:main",
        ],
    },
)

