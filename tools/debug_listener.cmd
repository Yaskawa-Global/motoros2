@echo off
:: debug_listener.cmd

:: SPDX-FileCopyrightText: 2022-2023, Yaskawa America, Inc.
:: SPDX-FileCopyrightText: 2022-2023, Delft University of Technology
::
:: SPDX-License-Identifier: Apache-2.0
cd %~dp0
python %~dp0\debug_listener.py
