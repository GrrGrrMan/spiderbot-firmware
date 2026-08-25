export interface LogEntry {
  date: string;
  tag: string;
  tagType: "research" | "build" | "fail" | "success" | "ai";
  title: string;
  desc: string;
}

export const LOGBOOK_ENTRIES: LogEntry[] = [
  {
    date: "30/04/26",
    tag: "RESEARCH",
    tagType: "research",
    title: "Initial Research on Spider Movement",
    desc: "Researched how multi-legged animals walk. Decided on a 6-legged (hexapod) design because 3 legs can stay on the ground while the other 3 move, forming a stable triangle so it never tips over."
  },
  {
    date: "01/05/26",
    tag: "PARTS",
    tagType: "build",
    title: "First Material Order Placed",
    desc: "Ordered our first batch of materials: basic servo motors, driver chips, and acrylic sheets to test the initial physical design."
  },
  {
    date: "08/05/26",
    tag: "TEST",
    tagType: "fail",
    title: "First Acrylic Leg Test & Mechanical Weakness",
    desc: "Cut our first leg pieces from acrylic and glued them together. When we turned the motor on, the acrylic flexed and the hot glue snapped. We learned acrylic is too brittle for load-bearing leg joints."
  },
  {
    date: "09/05/26",
    tag: "BUILD",
    tagType: "build",
    title: "Full Acrylic Chassis Completed",
    desc: "Cut out all 6 legs and a basic hexagon flat body to see the proportions. Confirmed that custom 3D printing is required for strength."
  },
  {
    date: "12/05/26",
    tag: "CAD",
    tagType: "build",
    title: "Transition to Onshape 3D Printing (PETG)",
    desc: "Switched completely to CAD in Onshape. Designed lightweight, high-strength brackets using PETG plastic."
  },
  {
    date: "23/05/26",
    tag: "CODE",
    tagType: "research",
    title: "First Motor Angle Controller",
    desc: "Wrote code to control the angle of each motor manually. Found that moving each joint by hand is too difficult for a human operator."
  },
  {
    date: "28/05/26",
    tag: "CAM",
    tagType: "build",
    title: "3D Printed Camera Eyes Holder",
    desc: "Designed and 3D printed a mounting head on the front of the body to hold an ESP32-CAM video streamer."
  },
  {
    date: "04/06/26",
    tag: "HARDWARE",
    tagType: "build",
    title: "Dual Microcontroller Upgrade (ESP32-S3)",
    desc: "The camera chip wasn't fast enough to handle both live video and motor math. Added an ESP32-S3 chip to act as a dedicated motor controller."
  },
  {
    date: "09/06/26",
    tag: "FAILURE",
    tagType: "fail",
    title: "Battery Explosion & Circuit Burnout",
    desc: "Tested 4x 18650 batteries together. The sudden electrical rush caused a resistor on the motor board to explode, burning out 3 servo motors. Added fuses and ordered stronger voltage regulators."
  },
  {
    date: "11/06/26",
    tag: "WEIGHT",
    tagType: "fail",
    title: "18650 Battery Cells Proved Too Heavy",
    desc: "Discovered that carrying 4 large batteries made the robot too heavy for the motors to lift."
  },
  {
    date: "16/06/26",
    tag: "MATH",
    tagType: "research",
    title: "Inverse Kinematics (IK) Math Breakthrough",
    desc: "Researched Inverse Kinematics. Instead of manually guessing each motor angle, we calculate the exact angles backwards based on where the foot needs to step on the ground."
  },
  {
    date: "20/06/26",
    tag: "CODE",
    tagType: "success",
    title: "IK Controller Version 2.0",
    desc: "Wrote our first working Inverse Kinematics math engine. Now, sending one target position automatically bends all 3 leg joints smoothly."
  },
  {
    date: "30/06/26",
    tag: "SENSORS",
    tagType: "build",
    title: "Multi-Sensor Suite Integrated",
    desc: "Coded 3 environmental sensors: Ultrasonic (distance sensing), Motion/Gyro (tilt balance), and Infrared (obstacle detection)."
  },
  {
    date: "13/07/26",
    tag: "CHASSIS",
    tagType: "build",
    title: "Redesigned Enclosed 3D Body",
    desc: "Replaced the flat plate body with a fully enclosed 3D chassis to protect sensitive internal wires and chips from drops and dust."
  },
  {
    date: "14/07/26",
    tag: "BRAIN",
    tagType: "build",
    title: "Raspberry Pi 5 Hub Integration",
    desc: "Connected a borrowed Raspberry Pi 5 to act as the main brain, coordinating the camera, Wi-Fi messages, and motor controller."
  },
  {
    date: "19/07/26",
    tag: "POWER",
    tagType: "success",
    title: "5.3V Regulated Power Tuning",
    desc: "Installed high-efficiency buck converters tuned precisely to 5.3V. This gave the servo motors maximum pushing power without overheating."
  },
  {
    date: "26/07/26",
    tag: "AI",
    tagType: "ai",
    title: "Cloud AI Model Integrated (Groq / Llama 3.3)",
    desc: "Connected a high-speed AI language model. The robot can now understand plain human spoken requests and convert them into walking steps."
  },
  {
    date: "03/08/26",
    tag: "STAND",
    tagType: "fail",
    title: "Total Mass Reached 1.3kg & Stand Presentation",
    desc: "With all electronics and sensors installed, total robot weight reached 1.3kg (too heavy for budget servos to walk across the floor). Designed a custom DTC stand so the robot can demonstrate full IK motion safely on external power."
  },
  {
    date: "09/08/26",
    tag: "AUDIO",
    tagType: "build",
    title: "Audio Speaker Added for Voice Feedback",
    desc: "Installed an I2S digital speaker on the robot so it can speak back to the user."
  },
  {
    date: "14/08/26",
    tag: "UI",
    tagType: "success",
    title: "Real-Time 3D Web Dashboard",
    desc: "Built a web dashboard that shows a live 3D visual of the robot's leg positions syncing in real time over Wi-Fi."
  },
  {
    date: "15/08/26",
    tag: "VISION",
    tagType: "ai",
    title: "AI Camera Eyes & Scene Detection",
    desc: "Enabled the AI camera to inspect surroundings, describe objects in front of it, and react to hand gestures."
  },
  {
    date: "20/08/26",
    tag: "RELEASE",
    tagType: "success",
    title: "Final Assembly & Science Fair Ready",
    desc: "Completed the final enclosed body, cable management, and stand presentation. All software and CAD files open-sourced for the judges!"
  }
];