/*
    Aerospace Decoder - Copyright (C) 2018 Bob Anderson (VK2GJ)

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef SYS_HELPER_H_
#define SYS_HELPER_H_

#define ARRAYSZ(x)  (sizeof(x) / sizeof((x)[0]))

#define MAX(x,y) ( \
    { __auto_type __x = (x); __auto_type __y = (y); \
      __x > __y ? __x : __y; })

#endif /* SYS_HELPER_H_ */
