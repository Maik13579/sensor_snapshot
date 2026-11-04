# Copyright 2026 Maik Knof
# Maintainer: Maik Knof <maik.knof@gmx.de>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# All filters must pass. Non-finite field values fail, including NOT_EQUAL.
uint8 LESS=0
uint8 LESS_OR_EQUAL=1
uint8 EQUAL=2
uint8 NOT_EQUAL=3
uint8 GREATER_OR_EQUAL=4
uint8 GREATER=5

# Exact PointField name, e.g. "z", "intensity", or "ring".
string field
uint8 comparison
float64 value
# Select an element of a field with count > 1. Zero selects its first element.
uint32 element_index
