# go_sim3

Программно-инструментальный комплекс для моделирования 
автономного шагающего робота Unitree Go2 в городской среде.

Проект разрабатывается для моделирования движения робота,
 навигации, одометрии и экспериментальных исследований задач курьерской доставки.

## YOLO demo

Опциональный YOLO-модуль читает камеру робота из `/robot1/color/image_raw`,
публикует annotated image в `/robot1/vision/annotated_image`, JSON detections в
`/robot1/vision/detections` и строки для GUI в `/robot1/vision/yolo_log`.

Положить модель:

```bash
mkdir -p models/yolo
cp runs/city_cv/yolo11n_city_cv_roboflow/weights/best.pt models/yolo/city_cv_yolo11n_best.pt
```

Установить Python-зависимости в окружение, из которого запускается ROS node.
Для ROS Jazzy и `cv_bridge` используйте NumPy 1.x и OpenCV до 4.12:

```bash
pip install "numpy<2" "opencv-python<4.12" ultralytics
```

Запуск базового стека с YOLO:

```bash
ros2 launch gazebo_sim launch.py enable_yolo:=true yolo_period_sec:=2.0 show_yolo_window:=true
```

Запуск без OpenCV-окна:

```bash
ros2 launch gazebo_sim launch.py enable_yolo:=true show_yolo_window:=false
```

Меньше нагрузка на ноутбук:

```bash
ros2 launch gazebo_sim launch.py enable_yolo:=true yolo_period_sec:=5.0 show_yolo_window:=false
```

Если YOLO установлен в отдельном виртуальном окружении, активируйте его после ROS:

```bash
source /opt/ros/jazzy/setup.bash
source ~/go_sim3/install/setup.bash
source ~/go_sim3/.venv-yolo/bin/activate
python3 -m go2_vision.yolo_detector_node --ros-args \
  -p enable_yolo:=true \
  -p inference_period_sec:=2.0 \
  -p show_yolo_window:=true
```

Если такое окружение конфликтует с ROS Python-пакетами, проще установить
Ultralytics в пользовательский Python:

```bash
pip install --user ultralytics opencv-python
```
