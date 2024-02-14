# deciLight - Visual Noise Monitor & Educational Tool

deciLight is a WIP traffic signal-inspired lighting system, designed to dynamically respond to ambient sound levels. This modular, stackable light features a unique capability to change colors—from red to yellow to green—based on the decibel levels in its surrounding environment. Users can set and adjust sensitivity thresholds remotely using an infrared (IR) remote control, which also allows for manual color and brightness changes, offering versatility and convenience.

### Classroom Noise Management

In the classroom, deciLight can serve as an effective noise monitor, signaling to students when the noise level has become too high (red), when it's close (yellow), or when the classroom environment is at an acceptable sound level (green). This visual cue helps in self-regulation, as students can adjust their volume without direct intervention from the teacher, fostering a sense of responsibility and self-awareness among the pupils. By setting specific decibel thresholds via the IR remote, educators can customize the sensitivity of deciLight to suit the needs of different activities, whether it's quiet reading time or a lively group discussion.

### Educational Games and Activities

Beyond its utility as a noise monitor, deciLight's color-changing feature can be integrated into various games and activities that engage students and facilitate learning. For instance:

- **Green Light, Red Light Game:** Leveraging deciLight's ability to change colors, teachers can conduct the classic "Green Light, Red Light" game, where students move when the light is green and stop when it's red. This can be a fun, physically active break between lessons or used as a tool for teaching self-control and listening skills.
- **Sound Level Challenges:** Teachers can create challenges for students to maintain a certain noise level (green) during group work or activities, rewarding them when they successfully stay within the acceptable range. This encourages teamwork and collective effort to achieve a common goal.
- **Interactive Storytelling:** Incorporating deciLight into storytelling can make reading sessions more interactive. For example, the light could change colors to reflect the mood or action within the story, or students could be asked to modulate their voice levels to keep the light green, enhancing engagement and comprehension.

### Bill of Materials (BOM):

- **ESP32 Microcontroller:** The brain of deciLight, offering WiFi and Bluetooth capabilities for future expansions and updates.
- **NeoPixel Jewel:** Provides bright, customizable colors for the light signal, ensuring vivid visibility.
- **IR Receiver:** Enables remote control functionality, allowing users to adjust settings and change colors from a distance.
- **I2S MEMS Microphone:** Senses ambient sound levels to trigger color changes based on predefined decibel thresholds.
- **Screws for Assembly:** m3x5 screws (4), m3-3 screws (2), and m1.5x3 screws (4-8) for secure assembly and mounting.
- **IR LED Remote:** Offers a user-friendly interface for adjusting deciLight settings and colors remotely.
- **5V 2A USB Power Supply:** Ensures reliable power delivery to the deciLight. A high-quality supply is recommended for optimal performance.
- **Wire:** Necessary for connections and assembly.

### Future Enhancements:

- **Networked Synchronization:** The ability to pair multiple deciLights (ESP-NOW?), creating a cohesive and synchronized lighting experience across multiple units.
- **External Display:** Show operational modes and noise thresholds in real-time.
- **Dampening:** Currently the switch between colors based on sound level is quite abrupt. I want to implement a method to smooth out the jump by taking into account a delay or somehow use data over time.
- **WiFi/Bluetooth control:** Adjust settings and modes via a mobile device.
- **Multi-device control:** Control multiple deciLights via single IR remote.

### Assembly & Printing Tips:

- The construction of deciLight is designed to be straightforward and user-friendly.
- The ESP32 mount is adjustable, catering to various models of the ESP32. If the mounting holes differ, customizing your mounting plate might be necessary.
- To enhance light reflection and efficiency, applying aluminum tape to the reflector is recommended.
- The components, including the reflector, lens, and visor, are designed for a friction fit, simplifying the assembly process.
- Print the reflector in vase-mode.
- Print the lens in clear PETG with grid infill.
- Print the feet in TPU and attach them with double sided tape.

### Schematic:


![Screenshot from 2024-02-13 23-50-47](https://github.com/bbbenji/deciLight/assets/1678118/5957b364-939a-45fc-963b-7a0aaaa96e0c)

### Firmware:

- The deciLight firmware, available on GitHub at [https://github.com/bbbenji/deciLight](https://github.com/bbbenji/deciLight), is open-source and provides a foundation for custom modifications and updates. I encouraged anyone to contribute to its development and customize their deciLight experience.
- For alternative functionality, consider flashing deciLight with WLED, offering sound-reactive animations at the sacrifice of precise decibel readings.
