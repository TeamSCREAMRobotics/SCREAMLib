# SCREAMLib
Common code/libraries for robot projects

To install the latest version of this library add the following to your ```build.gradle``` file:

   ```gradle
   allprojects {
       repositories {
           mavenLocal()
           jcenter()
           maven {url 'https://jitpack.io'}
       }
   }
   dependencies {
         implementation 'com.github.TeamSCREAMRobotics:SCREAMLib:main-SNAPSHOT'
   }
   configurations.all {
       resolutionStrategy.cacheChangingModulesFor 0, 'seconds'
   }
   ```

## Development

For development, clone this repository locally to make edits. To use the local version on dependent projects, use the following instead:

```gradle
implementation 'com.github.TeamSCREAMRobotics:SCREAMLib:local'
```
