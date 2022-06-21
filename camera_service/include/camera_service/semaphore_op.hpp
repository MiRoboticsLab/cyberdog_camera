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

#ifndef CAMERA_SERVICE__SEMAPHORE_OP_HPP_
#define CAMERA_SERVICE__SEMAPHORE_OP_HPP_

#include <sys/sem.h>

#include <iostream>

#define IPCKEY_PATH "/"

union semun {
  int val;    // value for SETVAL
  struct semid_ds * buf;   // buffer for IPC_STAT&IPC_SET
  unsigned short * array;   // array for GETALL&SETALL
  struct seminfo * __buf;   // buffer for IPC_INFO
};

inline int CreateSem(unsigned char proj_id, int sem_num, int & sem_set_id)
{
  // convert to a system ipc key
  key_t key = ftok(IPCKEY_PATH, proj_id);
  if (key < 0) {
    std::cout << "Convert to ipc key fail when create semaphore. " << std::endl;
    return -1;
  }

  // get the semaphore set id associated with a key
  sem_set_id = semget(key, sem_num, IPC_CREAT | 0666);
  if (sem_set_id < 0) {
    std::cout << "Get the semaphore id fail. " << std::endl;
    return -1;
  }
  std::cout << "Semaphore set id is: " << sem_set_id << std::endl;

  return 0;
}

inline int SetSemInitVal(int sem_set_id, int sem_index, int init_val)
{
  // set the initial value of the semaphore
  union semun sem_union;
  sem_union.val = init_val;
  if ((semctl(sem_set_id, sem_index, SETVAL, sem_union)) < 0) {
    std::cout << "Set the initial value fail, semaphore index: " << sem_index << std::endl;
    return -1;
  }

  return 0;
}

inline int WaitSem(int sem_set_id, int sem_index)
{
  struct sembuf sem_buf;
  sem_buf.sem_num = sem_index;
  sem_buf.sem_op = -1;
  sem_buf.sem_flg = SEM_UNDO;

  if (semop(sem_set_id, &sem_buf, 1) < 0) {
    std::cout << "Semaphore P operation fail. " << std::endl;
    return -1;
  }

  return 0;
}

inline int SignalSem(int sem_set_id, int sem_index)
{
  struct sembuf sem_buf;
  sem_buf.sem_num = sem_index;
  sem_buf.sem_op = 1;
  sem_buf.sem_flg = SEM_UNDO;

  if (semop(sem_set_id, &sem_buf, 1) < 0) {
    std::cout << "Semaphore V operation fail. " << std::endl;
    return -1;
  }

  return 0;
}

inline int DelSem(int sem_set_id)
{
  union semun sem_union;
  if (semctl(sem_set_id, 0, IPC_RMID, sem_union) < 0) {
    std::cout << "Del semaphore fail. " << std::endl;
    return -1;
  }
  std::cout << "Semaphore is deleted. " << std::endl;

  return 0;
}

#endif  // CAMERA_SERVICE__SEMAPHORE_OP_HPP_
