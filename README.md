# SCREAMLib
[![Build](https://github.com/TeamSCREAMRobotics/SCREAMLib/actions/workflows/maven-publish.yml/badge.svg?branch=main)](https://github.com/TeamSCREAMRobotics/SCREAMLib/actions/workflows/maven-publish.yml)
[![Github Page](https://img.shields.io/website?label=Page&logo=github&up_message=online&down_message=offline&url=https%3A%2F%2Fstremio.github.io%2Fstremio-web%2F)](https://teamscreamrobotics.github.io/SCREAMLib/)

Common code/libraries for robot projects

To install the latest version of this library, right click ```build.gradle``` and select ```Manage Vendor Libraries``` . Then click on ```Install new libraries (Online)``` and a textbox should appear. Paste in the URL then press enter.

```url
https://teamscreamrobotics.github.io/SCREAMLib/SCREAMLib.json
```

## Required Dependencies 
- CTRE Pheonix6
- Pathplanner
- Doglog

## Using the the lib locally
Replace {version} with the actual version

```gradle
implementation 'com.github.TeamSCREAMRobotics:SCREAMLib:{version}'
```
