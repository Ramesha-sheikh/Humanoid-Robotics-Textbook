# Module 4: Whisper for Commands

## Integrating OpenAI Whisper for Voice Commands in Robotics

Voice interfaces provide an intuitive and natural way for humans to interact with robots. OpenAI's Whisper is a versatile speech-to-text model that can accurately transcribe human speech, making it an excellent candidate for enabling voice commands in robotic applications. By integrating Whisper with ROS 2, robots can understand spoken instructions and translate them into actionable commands.

### Introduction to OpenAI Whisper:

Whisper is a pre-trained neural network for automatic speech recognition (ASR) developed by OpenAI. It's trained on a massive dataset of diverse audio and text, allowing it to perform robust transcription across multiple languages and accents, even with background noise. Its capabilities extend beyond simple transcription to include language identification and translation.

### Why Whisper for Robot Commands?

1.  **High Accuracy:** Whisper's performance in transcribing various forms of speech makes it reliable for interpreting commands.
2.  **Multilingual Support:** Enables robots to understand commands in different languages.
3.  **Robustness:** Performs well in challenging audio environments, which is common in real-world robotics.
4.  **Open-Source and Flexible:** Can be integrated into custom robotic systems, with various model sizes available to balance accuracy and computational resources.

### Integrating Whisper with ROS 2 for Voice Commands:

The typical pipeline involves:

1.  **Audio Acquisition:**
    *   Using a microphone connected to the robot (or a computer communicating with the robot).
    *   Publishing audio data as a ROS 2 topic (e.g., `audio_common_msgs/msg/AudioData`).

2.  **Whisper Transcription Node:**
    *   A ROS 2 node (e.g., a Python script using the `whisper` library) subscribes to the audio topic.
    *   It processes the audio data, feeds it to the Whisper model, and receives the transcribed text.
    *   The transcribed text is then published to another ROS 2 topic (e.g., `std_msgs/msg/String`).

3.  **Command Interpretation Node:**
    *   Another ROS 2 node subscribes to the transcribed text topic.
    *   This node is responsible for parsing the text and mapping it to specific robot actions or navigation goals.
    *   This could involve simple keyword matching, regular expressions, or more advanced Natural Language Understanding (NLU) techniques.
    *   Once a command is understood, it triggers corresponding ROS 2 actions or services (e.g., publishing velocity commands, calling a manipulation service).

### Conceptual ROS 2 Graph for Voice Commands:

```mermaid
graph LR
    A[Microphone] -- Raw Audio --> B(ROS 2 Audio Publisher Node)
    B -- /audio/data --> C{Whisper Transcription Node}
    C -- /speech/text --> D{Command Interpretation Node}
    D -- Robot Actions/Goals --> E[Robot Control System (e.g., Nav2)]
```

### Example: Simple Python ROS 2 Node for Command Interpretation (Conceptual):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandInterpreter(Node):

    def __init__(self):
        super().__init__('command_interpreter')
        self.subscription = self.create_subscription(
            String, '/speech/text', self.text_listener_callback, 10)
        self.publisher_vel = self.create_publisher(String, '/robot/cmd_vel', 10) # Example for publishing velocity
        self.get_logger().info('Command Interpreter Node has started.')

    def text_listener_callback(self, msg):
        transcribed_text = msg.data.lower()
        self.get_logger().info(f'Received: "{transcribed_text}"')

        if "move forward" in transcribed_text:
            self.get_logger().info('Executing: Move Forward')
            # Publish a command for moving forward
            vel_msg = String()
            vel_msg.data = "forward"
            self.publisher_vel.publish(vel_msg)
        elif "stop" in transcribed_text:
            self.get_logger().info('Executing: Stop')
            # Publish a stop command
            vel_msg = String()
            vel_msg.data = "stop"
            self.publisher_vel.publish(vel_msg)
        # Add more commands as needed

def main(args=None):
    rclpy.init(args=args)
    command_interpreter = CommandInterpreter()
    rclpy.spin(command_interpreter)
    command_interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Roman Urdu Explanation:

`Whisper ko robot ke liye voice commands ke liye istemal karna aik behtareen tareeqa hai. OpenAI ka Whisper model insan ki awaaz ko likhai mein badalta hai, jis se robot hamari baat samajh sakta hai. Ismein aap microphone se awaaz ko ROS 2 topic par publish karte hain, phir Whisper us awaaz ko likhai mein badal kar aik aur ROS topic par bhejta hai. Phir aik doosra ROS node us likhai ko parh kar robot ke liye commands banata hai, jaise "aage chalo" ya "ruko".`

### Multiple Choice Questions (MCQs):

1.  **What is the primary function of OpenAI's Whisper model in a robotics context?**
    a) Robot navigation.
    b) Speech-to-text transcription.
    c) Object detection.
    d) Reinforcement learning.
    *Correct Answer: b) Speech-to-text transcription.*

2.  **Why is Whisper considered suitable for robot voice commands?**
    a) It only understands a single language.
    b) It offers high accuracy and robustness in varied audio environments.
    c) It directly controls robot actuators.
    d) It requires minimal computational resources.
    *Correct Answer: b) It offers high accuracy and robustness in varied audio environments.*

3.  **In the voice command pipeline, what is the role of the 'Command Interpretation Node'?**
    a) To acquire raw audio from the microphone.
    b) To transcribe speech into text using Whisper.
    c) To parse transcribed text and map it to specific robot actions.
    d) To publish audio data to a ROS 2 topic.
    *Correct Answer: c) To parse transcribed text and map it to specific robot actions.*

4.  **Which ROS 2 message type would typically be used to publish transcribed text from Whisper?**
    a) `audio_common_msgs/msg/AudioData`
    b) `sensor_msgs/msg/JointState`
    c) `std_msgs/msg/String`
    d) `geometry_msgs/msg/Twist`
    *Correct Answer: c) `std_msgs/msg/String`*

5.  **Which of the following is NOT typically a part of the voice command integration pipeline?**
    a) Audio Acquisition.
    b) Command Interpretation.
    c) Whisper Transcription.
    d) Direct hardware control without any ROS 2 layer.
    *Correct Answer: d) Direct hardware control without any ROS 2 layer.*

### Further Reading:
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [ROS 2 Audio Common](https://github.com://ros-perception/audio_common)
- [ROS 2 Python Client Library (rclpy) Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-tools/Writing-A-Simple-Publisher-And-Subscriber-Python.html)