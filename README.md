## step1
  新开终端在**根目录**clone该仓库:
  ```
  git clone https://github.com/Koi-Blue/Vision1.1.git
  ```
  执行：
  ```
  cd Vision1.1
  ```
  ```
  mkdir Vision_tol_50/build && mkdir Vision_tol_150/build && mkdir Vision_tol_250/build && mkdir Vision_tol_350/build && mkdir Vision_tol_500/build
  ```

## step2
  分别进入**目录** & **构建** & **编译**
  ```
  cd ~/Vision1.1/Vision_tol_50/build
  ```
  ```
  cmake ..
  ```
  ```
  make
  ```
  此时构建& 编译完50容错的任务流，对于其他容错任务，第一步分别执行：
  
  tol_150:
  ```
  cd ~/Vision1.1/Vision_tol_150/build
  ```
  tol_250:
  ```
  cd ~/Vision1.1/Vision_tol_250/build
  ```
  tol_350:
  ```
  cd ~/Vision1.1/Vision_tol_350/build
  ```
  tol_500:
  ```
  cd ~/Vision1.1/Vision_tol_500/build
  ```
  第二步和第三步都是：
  ```
  cmake ..
  ```
  ```
  make
  ```

## step3
  编译完成，执行可执行文件
  **Attention：新开终端，在根目录直接执行**
  ```
  ./Vision1.1/Vision_tol_50/build/vs
  ```
  tol_150:
  ```
  ./Vision1.1/Vision_tol_150/build/vs
  ```
  tol_250:
  ```
  ./Vision1.1/Vision_tol_250/build/vs
  ```
  tol_350:
  ```
  ./Vision1.1/Vision_tol_350/build/vs
  ```
  tol_500:
  ```
  ./Vision1.1/Vision_tol_500/build/vs
  ```
