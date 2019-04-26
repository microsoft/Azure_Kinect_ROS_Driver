# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

message("-- Custom K4A install -- ")
file(MAKE_DIRECTORY "${DLL_COPY_DIRECTORY}")
message("Installing to: ${DLL_COPY_DIRECTORY}")

foreach(DLL ${K4A_DLL_FILES})
    message("Copying DLL from ${DLL} to ${DLL_COPY_DIRECTORY}")
    file(COPY "${DLL}" DESTINATION "${DLL_COPY_DIRECTORY}")
endforeach(DLL)
message("-- Custom K4A install finished -- ")