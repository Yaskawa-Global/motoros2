#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2023, Yaskawa America, Inc.
# SPDX-FileCopyrightText: 2023, Delft University of Technology
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import argparse
import re
import sys

from dataclasses import dataclass
from mistletoe import (
    ast_renderer,
    Document,
)
from pycparser import (
    c_ast,
    parse_file,
)


ERROR_HANDLING_H = "ErrorHandling.h"
TROUBLESHOOTING_MD = "Troubleshooting.md"
MAIN_ALARM_ENUM = "ALARM_MAIN_CODE"
CATCHALL_RANGE_MARKER = "[xx]"
ALARM_HEADING_PREFIX = "Alarm:"
SC_RANGE_MAX = 65535
SC_RANGE_MIN = 0


@dataclass
class SubcodeDef:
    # fi: ALARM_TASK_CREATE_FAIL
    alarm_name: str
    # fi: 8010
    alarm: int
    # fi: SUBCODE_INCREMENTAL_MOTION
    subcode_name: str
    # fi: 2
    subcode: int
    # fi: src/ErrorHandling.h:92:5
    loc_str: str

    def __repr__(self) -> str:
        return f"{self.alarm}[{self.subcode}]"


@dataclass
class AlarmWithSubcodeRange:
    code: int
    sc_s: int
    sc_e: int

    @property
    def is_catch_all(self) -> bool:
        return self.sc_s == SC_RANGE_MIN and self.sc_e == SC_RANGE_MAX

    def __repr__(self) -> str:
        # special repr for catch-alls
        if self.is_catch_all:
            return f"{self.code:4d}{CATCHALL_RANGE_MARKER}"
        # range
        if self.sc_e > self.sc_s:
            return f"{self.code:4d}[{self.sc_s}-{self.sc_e}]"
        # regular, singular, subcode
        return f"{self.code:4d}[{self.sc_s}]"

    def __contains__(self, val):
        return val in range(self.sc_s, self.sc_e + 1)


def main():
    parser = argparse.ArgumentParser(
        description="Tries to find headings in MD which document alarms+subcodes "
        "defined in HEADER",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-a",
        "--check-all",
        action="store_true",
        help="Check all subcodes, don't immediately exit",
    )
    parser.add_argument(
        "-i",
        "--ignore-enum",
        action="append",
        default=[],
        dest="enums_to_ignore",
        help=f"Enum(s) in '{ERROR_HANDLING_H}' to ignore. Option may be repeated.",
        metavar="ENUM",
        type=str,
    )
    parser.add_argument(
        "--no-catch-alls",
        action="store_true",
        help=f"Ignore '{CATCHALL_RANGE_MARKER}' subcode ranges in '{TROUBLESHOOTING_MD}'",
    )
    parser.add_argument(
        "--warn-catch-alls",
        action="store_true",
        help=f"Warn about subcodes documented by catch-all ranges (ie: '{CATCHALL_RANGE_MARKER}')",
    )
    parser.add_argument("hdr_file", metavar="HDR", help=f"Path to '{ERROR_HANDLING_H}'")
    parser.add_argument("md_file", metavar="MD", help=f"Path to '{TROUBLESHOOTING_MD}'")
    args = parser.parse_args()

    # retrieve subcode defs from C header file
    subcode_defs: list[SubcodeDef] = get_subcode_defs(
        header_file=args.hdr_file, enums_to_ignore=args.enums_to_ignore
    )

    # retrieve subcode documentation from Markdown document
    alarm_docs: list[AlarmWithSubcodeRange] = get_alarm_docs(
        md_file=args.md_file, no_catch_alls=args.no_catch_alls
    )

    # for each subcode enum member, check there's a troubleshooting section documenting it
    ret = 0
    for sdef in subcode_defs:
        doc_range = find_subcode_doc(
            code=sdef.alarm,
            subcode=sdef.subcode,
            ranges=alarm_docs,
        )
        if doc_range:
            if doc_range.is_catch_all and args.warn_catch_alls:
                print(f"{sdef.loc_str}: warning: documented by catch-all '{doc_range}'")
        else:
            ret = 1
            print(f"{sdef.loc_str}: error: no documentation for '{sdef}' in '{args.md_file}'")
            if not args.check_all:
                break

    # done
    sys.exit(ret)


def get_subcode_defs(header_file: str, enums_to_ignore: list[str]) -> list[SubcodeDef]:
    # load C header and parse all 'typedef enums' in it
    # NOTE: we define 'BOOL' as 'int' for the two prototypes using it
    ast = parse_file(header_file, use_cpp=True, cpp_args=["-E", r"-DBOOL=int"])
    typedefd_enums: dict[str, c_ast.Node] = find_typedefd_enums(ast)

    # remove things we don't need (if enums_to_ignore is empty, this just won't do anything)
    typedefd_enums = {k: v for k, v in typedefd_enums.items() if not k in enums_to_ignore}

    # use main alarm code dict to coordinate parsing of the others
    main_alarm_codes_dict: dict[str, int] = ast_enum_to_dict(typedefd_enums[MAIN_ALARM_ENUM])

    # convert all other enums to lists of subcode defs
    subcode_defs: list[SubcodeDef] = []
    for name, code in main_alarm_codes_dict.items():
        alm_ast_node = typedefd_enums[f"{name}_SUBCODE"]
        subcode_defs.extend(ast_enum_to_list(node=alm_ast_node, alarm_name=name, alarm_code=code))
    return subcode_defs


class EnumTypeDeclVisitor(c_ast.NodeVisitor):
    """Extracts all typedef-d enums from the AST"""

    def __init__(self):
        self._enums = {}

    def visit_TypeDecl(self, node):
        if type(node.type) == c_ast.Enum:
            self._enums[node.declname] = node.type


def find_typedefd_enums(node: c_ast.Node) -> dict[str, c_ast.Enum]:
    v = EnumTypeDeclVisitor()
    v.visit(node)
    return v._enums


def ast_enum_to_dict(node: c_ast.Enum) -> dict[str, int]:
    ret = {}
    val = 0
    for memb in node.values.enumerators:
        if memb.value and type(memb.value) == c_ast.Constant:
            # we only support integer enums (and so does C)
            assert memb.value.type == "int"
            val = int(memb.value.value)
        ret[memb.name] = val
        # either there is a value defined for the next member, or not.
        # if not, just use the current value but increment it by one
        val += 1
    return ret


def ast_enum_to_list(node: c_ast.Enum, alarm_name: str, alarm_code: int) -> list[SubcodeDef]:
    ret = []
    val = 0
    for memb in node.values.enumerators:
        if memb.value and type(memb.value) == c_ast.Constant:
            # we only support integer enums (and so does C)
            assert memb.value.type == "int"
            val = int(memb.value.value)
        ret.append(
            SubcodeDef(
                alarm_name=alarm_name,
                alarm=alarm_code,
                subcode_name=memb.name,
                subcode=val,
                loc_str=str(memb.coord),
            )
        )
        # either there is a value defined for the next member, or not.
        # if not, just use the current value but increment it by one
        val += 1
    return ret


def get_alarm_docs(md_file: str, no_catch_alls: bool) -> list[AlarmWithSubcodeRange]:
    # load Md doc with all alarm codes we document
    with open(md_file, "r") as fin:
        d = Document(fin.readlines())
    md_doc = ast_renderer.get_ast(d)
    headings = collapse_heading_children(get_headings_at_lvl(md_doc))

    # keep only those headings with text of the form "Alarm: nnnn[..]"
    headings = [h for h in headings if h.startswith(ALARM_HEADING_PREFIX)]

    # if requested, filter out 'catch-all' ranges (ie: '[xx]')
    if no_catch_alls:
        headings = [h for h in headings if not CATCHALL_RANGE_MARKER in h]

    # parse them into tuples
    return [parse_alarm_code_with_subcode(h) for h in headings]


def get_headings_at_lvl(doc, level=3):
    def predicate(c):
        return c["type"] == "Heading" and c["level"] == level

    return [c for c in doc["children"] if predicate(c)]


def collapse_heading_children(headings):
    return [c["children"][0]["content"] for c in headings]


alm_re = re.compile(r"\b(?:(\d{4})\[(?:(\d+)|(xx)|(\d+) \- (\d+))\])")


def parse_alarm_code_with_subcode(subcode_spec) -> AlarmWithSubcodeRange:
    """
    turns "8010[1]", "8010[xx]" or "8010[1 - 2]"
    into
    (8010, 1, 1), (8010, 0, 65535) or (8010, 1, 2)
    """
    groups = alm_re.findall(subcode_spec)
    code, subcode_n, subcode_a, sc_range_s, sc_range_e = groups[0]

    def to_subcode_range(code, r_s, r_e):
        return AlarmWithSubcodeRange(int(code), int(r_s), int(r_e))

    if code and sc_range_s and sc_range_e:
        return to_subcode_range(code, sc_range_s, sc_range_e)
    if code and subcode_a:
        return to_subcode_range(code, SC_RANGE_MIN, SC_RANGE_MAX)
    if code and subcode_n:
        return to_subcode_range(code, subcode_n, subcode_n)
    raise ValueError(f"Failed to parse range spec: '{subcode_spec}'")


def find_subcode_doc(
    code: int, subcode: int, ranges: list[AlarmWithSubcodeRange]
) -> AlarmWithSubcodeRange:
    # naive O(n) 'search'
    for subcode_range in ranges:
        if code == subcode_range.code and subcode in subcode_range:
            return subcode_range
    return None


if __name__ == "__main__":
    main()
