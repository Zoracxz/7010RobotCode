# Show me tic tac toe

A virtual player that can use computer camera to observe state and make move in a popular Tic Tac Toe game. 

## Features
- recognizing state in the real-time using copmuter camera (examples in files: out.png, out2.png)
- determining next move
- communicating with the user using Text-to-Speech
- handling different games' strategies

## Setup
```
virtualenv venv
. venv/bin/activate
pip install -r requirements.txt
```

```

cd "D:\UM\WQF7010 ROBOTICS AND AUTOMATION\Project\show_me_tic_tac_toe" 

python -m venv venv

.\venv\Scripts\activate

& "D:\UM\WQF7010 ROBOTICS AND AUTOMATION\Project\show_me_tic_tac_toe\venv\Scripts\python.exe"  -m pip install --upgrade pip

& "D:\UM\WQF7010 ROBOTICS AND AUTOMATION\Project\show_me_tic_tac_toe\venv\Scripts\python.exe" -m pip install -r requirements.txt

python main.py
```

```
roscore
rosrun sound_play soundplay_node.py
python main.py
```
