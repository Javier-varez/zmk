# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

if (NOT CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
 	target_sources(app PRIVATE service.c)
 	target_sources(app PRIVATE common.c)
endif()

if (CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
 	target_sources(app PRIVATE central.c)
 	target_sources(app PRIVATE common.c)
 endif()
