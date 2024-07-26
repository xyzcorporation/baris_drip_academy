## 🛠 바리스 드립 명령어   

| no | command      | par1 | par2 | par3 | par4 | par5 | description      |
| -- | ------------ | ---- | ---- | ---- | ---- | ---- | ---------------- |
| 1  | HOME_NORMAL  | 0    | 0    | 0    | 0    | 0    | 로봇 기본 위치         |
| 2  | GRIPPER_INIT | 0    | 0    | 0    | 0    | 0    | 그리퍼 활성화          |
| 3  | PICKUP       | DSP  | 2    | 0    | 0    | 0    | 드리퍼 잡기           |
| 4  | HOLD         | COF  | 0    | 0    | 0    | 0    | 커피 머신으로 가기       |
| 5  | COFFEE_ON    |      |      |      |      |      | 커피 동작 - 종료       |
| 6  | UNHOLD       | COF  | 0    | 0    | 0    | 0    | 커피 머신에서 나오기      |
| 7  | HOME_NORMAL  | 0    | 0    | 0    | 0    | 0    | 로봇 기본 위치         |
| 8  | FLATTENING   | ZON  | 1    | 0    | 0    | 0    | 커피 흔들기           |
| 9  | PLACE_DRIP   | ZON  | 1    | 0    | 0    | 0    | 드리퍼 드립존에 놓기      |
| 10 | PICKUP       | DSP  | 1    | 0    | 0    | 0    | 컵 잡기             |
| 11 | PLACE_CUP    | ZON  | 1    | 0    | 0    | 0    | 컵 드립존에 놓기        |
| 12 | PICKUP       | KET  | 0    | 0    | 0    | 0    | 주전자 잡으러 가기       |
| 13 | WATER_TOGGLE |      |      |      |      |      | 모아이 동작 - 종료      |
| 14 | DRAIN_FIT    | HOT  | 1    | DPO  | 0    | 0    | 주전자 안에 있는 물양 맞추기 |
| 15 | HOME_KETTLE  | 0    | 0    | 0    | 0    | 0    | 주전자 잡은 로봇 기본 위치  |
| 16 | DRIP         | DPO  | SOL  | HOT  | 1    | 1    | 드립하기             |
| 17 | HOME_KETTLE  | 0    | 0    | 0    | 0    | 0    | 주전자 잡은 로봇 기본 위치  |
| 18 | DRAIN_ALL    | 0    | 0    | 0    | 0    | 0    | 주전안에 남은 물 다 버리기  |
| 19 | PLACE        | KET  | 0    | 0    | 0    | 0    | 주전자 내려놓기         |
| 20 | HOME_NORMAL  | 0    | 0    | 0    | 0    | 0    | 로봇 기본 위치         |
| 21 | PICKUP_CUP   | ZON  | 1    | 0    | 0    | 0    | 드립존에 있는 컵 잡기     |
| 22 | PLACE        | PIC  | 1    | 0    | 0    | 0    | 컵 드립존에 놓기        |
| 23 | PICKUP_DRIP  | ZON  | 1    | 0    | 0    | 0    | 드립존에 있는 드리퍼 들기   |
| 24 | PLACE        | BIN  | 0    | 0    | 0    | 0    | 드리퍼 쓰레기통에 버리기    |
| 25 | GESTURE      | ETC  | 2    | 0    | 0    | 0    | 인사               |
