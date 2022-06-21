// Copyright (c) 2021 Xiaomi Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CYBERDOG_VISION__SHARED_MEMORY_OP_HPP_
#define CYBERDOG_VISION__SHARED_MEMORY_OP_HPP_

#include <string.h>
#include <sys/shm.h>

#include <iostream>

#define IPCKEY_PATH "/"
#define IMAGE_SIZE 640 * 480 * 3

struct SharedImage
{
  double img_stamp;
  unsigned char img_data[IMAGE_SIZE];
};

inline int CreateShm(unsigned char proj_id, size_t size, int & shm_id)
{
  key_t key = ftok(IPCKEY_PATH, proj_id);
  if (key < 0) {
    std::cout << "Convert to ipc key fail when create shared memory. " << std::endl;
    return -1;
  }

  shm_id = shmget(key, size, IPC_CREAT | 0666);
  if (shm_id < 0) {
    std::cout << "Get the shared memory id fail. " << errno << std::endl;
    return -1;
  }

  return 0;
}

inline char * GetShmAddr(int shm_id, size_t size)
{
  char * addr = (char *)shmat(shm_id, NULL, 0);
  if (addr == (char *)-1) {
    std::cout << "Get the shared memory addr fail. " << std::endl;
    return nullptr;
  }
  memset(addr, 0, size);

  return addr;
}

inline int DelShm(int shm_id)
{
  if (shmctl(shm_id, IPC_RMID, NULL) < 0) {
    std::cout << "Remove the shared memory fail. " << std::endl;
    return -1;
  }

  return 0;
}

inline int DetachShm(char * shm_addr)
{
  if (shmdt(shm_addr) < 0) {
    std::cout << "Detach the shared memory fail. " << std::endl;
    return -1;
  }
  return 0;
}

#endif  // CYBERDOG_VISION__SHARED_MEMORY_OP_HPP_
