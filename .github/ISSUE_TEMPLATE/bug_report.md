---
name: Bug report
about: Create a report to help us improve
title: ''
labels: bug, triage needed
assignees: ''

---

<!--
Before filing a bug
- Ensure the bug reproduces on the latest commit of your release branch.
- Search existing issues and make sure this issue is not already filed.
-->

**Describe the bug**
A clear and concise description of what the bug is.

**To Reproduce**
Steps to reproduce the behavior:
1. Go to '...'
2. Click on '....'
3. Scroll down to '....'
4. See error

**Expected behavior**
A clear and concise description of what you expected to happen.

**Logs**
Please provide relevant logs to help diagnose your issue. To assist in debugging, please enable *info* level logging of the Azure Kinect Sensor SDK and attach any logs you have to this issue.
See [k4atypes.h](https://github.com/Microsoft/Azure-Kinect-Sensor-SDK/blob/feecae0456511ac734287571b101d10fd7292673/include/k4a/k4atypes.h#L184) for how to enable logs.

Please also enable *DEBUG* level logging of the ROS node.
See [roscpp/Overview/Logging](http://wiki.ros.org/roscpp/Overview/Logging) for how to enable ROS DEBUG logs.

**Screenshots**
If applicable, add screenshots to help explain your problem.

**Desktop (please complete the following information):**
 - OS: [e.g. Windows 10, Ubuntu]
 - Version [e.g. Version 1903, 18.04]

**Additional context**
Add any other context about the problem here.
