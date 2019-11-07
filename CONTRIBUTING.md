# Contributing to the Azure Kinect ROS Driver

The Azure Kinect ROS Driver team welcomes community feedback and contributions. This repo
is relatively new and team members are actively defining and refining the process. Feel free to point out any
discrepancies between documented process and the actual process.

## Reporting issues and suggesting new features

If the Azure Kinect ROS Driver is not working the way you expect it to, then please report that with a GitHub Issue.

### Filing a bug

Please review the list of open Issues to see if one is already open. Please review all categories, Bugs and
Enhancements. Also check for closed Issues before opening a new one.

When opening a new issue be sure to document:

* Steps to reproduce the error
* Expected results
* Actual results
* SDK version
* Firmware version
* Log Results (stdout / roserr) from the repro (if possible)

### Requesting new features

Please review the list of open Issues to see if one is already open. Please review all categories, Bugs and
Enhancements. Also check for Closed Issues before opening a new one.

## Finding issues you can help with

Looking for something to work on? Issues marked [``Good First Issue``](https://github.com/microsoft/Azure_Kinect_ROS_Driver/labels/good%20first%20issue)
are a good place to start.

You can also check the [``Help Wanted``](https://github.com/microsoft/Azure_Kinect_ROS_Driver/labels/help%20wanted) tag to
find other issues to help with. If you're interested in working on a fix, leave a comment to let everyone know and to help
avoid duplicated effort from others.

Once you are committed to fixing an issue, assign it to yourself so others know the issue has an owner.

## Contributing code changes

We welcome your contributions, especially to fix bugs and to make improvements which address the top Issues. Some general
guidelines:

* **DO** create one pull request per Issue, and ensure that the Issue is linked in the pull request.
* **DO** follow the [ROS style guide](http://wiki.ros.org/CppStyleGuide) guidelines, and keep code changes as small as possible.
* **DO** check for additional occurrences of the same problem in other parts of the codebase before submitting your PR.
* **DO** [link the Issue](https://github.com/blog/957-introducing-issue-mentions) you are addressing in the pull request.
* **DO** write a good description for your pull request. More detail is better. Describe why the change is being made and
why you have chosen a particular solution. Describe any manual testing you performed to validate your change.
* **DO NOT** submit a PR unless it is linked to an Issue marked Triage Approved. This enables us to have a discussion on
the idea before anyone invests time in an implementation.
* **DO NOT** merge multiple changes into one PR unless they have the same root cause.

**NOTE:** *Submitting a pull request for an approved Issue is not a guarantee it will be approved.*

# Making changes to the code

## Building
Check out how to set up your environment and do a build [here](docs/building.md).

## Style Guidelines
The node and nodelet are written in C++, and follow the [ROS style guide](http://wiki.ros.org/CppStyleGuide). Any changes should also follow the ROS style guide to maintain consistency with other ROS software and with the existing Azure Kinect ROS Driver codebase.
Please run clang-format with the [ROS style file](https://github.com/davetcoleman/roscpp_code_format) on your code to ensure formatting and whitespace is correct (which is also included in the package).

## Workflow for Submitting a Change

Like many ROS projects, Azure Kinect ROS Driver uses branches to keep track of versions of the project which are compatible with different ROS releases. These ROS release branches can be considered a `develop` branch like you would find on other projects. The ROS release branches should always be in a healthy state.

To submit a change to the Azure Kinect ROS Driver, follow this process:

1) Start with an issue that has been tagged with Triage Approved. Otherwise, create an Issue and
start a conversation with the Azure Kinect Team to get the Issue Triage Approved.
1) If you have not already, fork the repo.
1) Make changes.
1) Test the change using Azure Kinect hardware on a live ROS system.
1) Create a pull request.
   * The PR description must reference the issue.
1) An Azure Kinect team member will review the change. See the [review process](#review-process) for more information.
   * 1 team member must sign off on the change.
   * Other reviewers are welcome.
1) Once the change is signed off by a team member, an Azure Kinect team member will complete the PR.

**NOTE:** *Any update to the pull request after approval requires additional review.*

When completing a pull request, we will generally squash your changes into a single commit. Please
let us know if your pull request needs to be merged as separate commits.

## Review Process
After submitting a pull request, members of the Azure Kinect team will review your code. We will
assign the request to an appropriate reviewer. Any member of the community may
participate in the review, but at least one member of the team will ultimately approve
the request.

Often, multiple iterations will be needed to respond to feedback from reviewers. Try looking at
[past pull requests](https://github.com/Microsoft/Azure_Kinect_ROS_Driver/pulls?q=is%3Apr+is%3Aclosed) to see what the
experience might be like.

After at least one member of the Azure Kinect team has approved your request, it will be merged by a member of the Azure
Kinect team. For Pull Requests where feedback is desired but the change is not complete, please mark the Pull Request
as a draft.

# Releasing
For information on how to release a new version of the Azure Kinect ROS Driver, please see the [releasing guide](releasing.md).

# Contributor License Agreement
Most contributions require you to agree to a Contributor License Agreement (CLA) declaring that you have
the right to, and actually do, grant us the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether
you need to provide a CLA and decorate the PR appropriately (e.g., label,
comment). Simply follow the instructions provided by the bot. You will only
need to do this once across all repositories using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or contact
[opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.
