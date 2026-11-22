from setuptools import find_packages, setup

setup(
    name="simple_flight_payload",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/simple_flight_payload"]),
        ("share/simple_flight_payload", ["package.xml"]),
        ("share/simple_flight_payload/config", ["config/doors.json"]),
        ("share/simple_flight_payload/scripts", ["scripts/competitionctl.sh", "scripts/rebuild_competition.sh",
                                                "scripts/recording_name.py", "scripts/export_recordings.ps1"]),
    ],
    install_requires=["setuptools"],
    maintainer="simple-flight",
    maintainer_email="maintainer@example.com",
    description="Independent competition payload door commands via MAVROS.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["doorctl = simple_flight_payload.door:main"]},
)
