# Feature Specification: Physical AI & Humanoid Robotics Textbook Hackathon

**Feature Branch**: `001-robotics-textbook`
**Created**: 2025-12-01
**Status**: Draft
**Input**: User description: "Hackathon I: Create a Textbook for Teaching Physical AI & Humanoid Robotics Course. The future of work will be a partnership between people, intelligent agents (AI software), and robots. This shift won't necessarily eliminate jobs but will change what humans do, leading to a massive demand for new skills. We have already written a book on AI agents. Therefore, we want you to write a textbook to teach a course in Physical AI & Humanoid Robotics (The course details are documented below). Excel in the Hackathon and Launch Your Journey as an AI Startup Founder =€ Weve recently launched Panaversity (panaversity.org), an initiative focused on teaching cutting-edge AI courses. Alongside this, were working on publishing our first book, which you can explore at ai-native.panaversity.org. Our next milestone is to build a portal where authors can create AI-native technical textbooks, and readers can easily access and learn from them using AI Agents. We also plan to publish O/A Level, Science, Engineering, and Medical AI-native books to support students and professionals across disciplines. If you perform well in this hackathon, you may be invited for an interview to join the Panaversity core team and potentially step into the role of a startup founder within this growing ecosystem. You will get a chance to work with Panaversity founders Zia, Rehan, Junaid, and Wania and become the very best. You may also get a chance to teach at Panaversity, PIAIC, and GIAIC. Requirements You are required to complete a unified book project using Claude Code and Spec-Kit Plus. The core deliverables are: 1. AI/Spec-Driven Book Creation: Write a book using Docusaurus and deploy it to GitHub Pages. You will use Spec-Kit Plus ( https://github.com/panaversity/spec-kit-plus/ ) and Claude Code ( https://www.claude.com/product/claude-code ) to write the book. 2. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user. 3. Participants will receive points out of 100, for base functionality defined above. 4. Participants can earn up to 50 extra bonus points by creating and using reusable intelligence via Claude Code Subagents and Agent Skills in the book project. 5. Participants can receive up to 50 extra bonus points if they also implement Signup and Signin using https://www.better-auth.com/ At signup you will ask questions from the user about their software and hardware background. Knowing the background of the user we will be able to personalize the content. 6. Participants can receive up to 50 extra bonus points if the logged user can personalise the content in the chapters by pressing a button at the start of each chapter. 7. Participants can receive up to 50 extra bonus points if the logged user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter. Timeline Submission Deadline: Sunday, Nov 30, 2025 at 06:00 PM (form will close) Live Presentations: Sunday, Nov 30, 2025 starting at 6:00 PM on Zoom Top submissions will be invited via WhatsApp to present live on Zoom. Note: All submissions will be evaluated. Live presentation is by invitation only, but does not affect final scoring. Submit and Present Your Project: Once you have completed the project you will submit your project here: https://forms.gle/CQsSEGM3GeCrL43c8 Submit the following via the form: Public GitHub Repo Link Published Book Link for Github Pages or Vercel. Include a demo video link (must be under 90 seconds). Judges will only watch the first 90 seconds. You can use NotebookLM or record your demo. WhatsApp number (top submissions will be invited to present live) Everyone is welcome to join the Zoom meeting to watch the presentations. Only invited participants will present their submissions. Meeting starts at 6:00 PM on Sunday, Nov 30: Join Zoom Meeting Time: Nov 30, 2025 06:00 PM https://us06web.zoom.com/j/84976847088?pwd=Z7t7NaeXwVmmR5fysCv7NiMbfbhIda.1 Meeting ID: 849 7684 7088 Passcode: 305850 The Course Details Physical AI & Humanoid Robotics Focus and Theme: AI Systems in the Physical World. Embodied Intelligence. Goal: Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments. Quarter Overview The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces Physical AIAI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac. Module 1: The Robotic Nervous System (ROS 2) Focus: Middleware for robot control. ROS 2 Nodes, Topics, and Services. Bridging Python Agents to ROS controllers using rclpy. Understanding URDF (Unified Robot Description Format) for humanoids. Module 2: The Digital Twin (Gazebo & Unity) Focus: Physics simulation and environment building. Simulating physics, gravity, and collisions in Gazebo. High-fidelity rendering and human-robot interaction in Unity. Simulating sensors: LiDAR, Depth Cameras, and IMUs. Module 3: The AI-Robot Brain (NVIDIA Isaac") Focus: Advanced perception and training. NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation. Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation. Nav2: Path planning for bipedal humanoid movement. Module 4: Vision-Language-Action (VLA) Focus: The convergence of LLMs and Robotics. Voice-to-Action: Using OpenAI Whisper for voice commands. Cognitive Planning: Using LLMs to translate natural language ("Clean the room") into a sequence of ROS 2 actions. Capstone Project: The Autonomous Humanoid. A final project where a simulated robot receives a voice command, plans a path, navigates obstacles, identifies an object using computer vision, and manipulates it. Why Physical AI Matters Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space. Learning Outcomes Understand Physical AI principles and embodied intelligence Master ROS 2 (Robot Operating System) for robotic control Simulate robots with Gazebo and Unity Develop with NVIDIA Isaac AI robot platform Design humanoid robots for natural interactions Integrate GPT models for conversational robotics Weekly Breakdown Weeks 1-2: Introduction to Physical AI Foundations of Physical AI and embodied intelligence From digital AI to robots that understand physical laws Overview of humanoid robotics landscape Sensor systems: LIDAR, cameras, IMUs, force/torque sensors Weeks 3-5: ROS 2 Fundamentals ROS 2 architecture and core concepts Nodes, topics, services, and actions Building ROS 2 packages with Python Launch files and parameter management Weeks 6-7: Robot Simulation with Gazebo Gazebo simulation environment setup URDF and SDF robot description formats Physics simulation and sensor simulation Introduction to Unity for robot visualization Weeks 8-10: NVIDIA Isaac Platform NVIDIA Isaac SDK and Isaac Sim AI-powered perception and manipulation Reinforcement learning for robot control Sim-to-real transfer techniques Weeks 11-12: Humanoid Robot Development Humanoid robot kinematics and dynamics Bipedal locomotion and balance control Manipulation and grasping with humanoid hands Natural human-robot interaction design Week 13: Conversational Robotics Integrating GPT models for conversational AI in robots Speech recognition and natural language understanding Multi-modal interaction: speech, gesture, vision Assessments ROS 2 package development project Gazebo simulation implementation Isaac-based perception pipeline Capstone: Simulated humanoid robot with conversational AI Hardware Requirements This course is technically demanding. It sits at the intersection of three heavy computational loads: Physics Simulation (Isaac Sim/Gazebo), Visual Perception (SLAM/Computer Vision), and Generative AI (LLMs/VLA). Because the capstone involves a "Simulated Humanoid," the primary investment must be in High-Performance Workstations. However, to fulfill the "Physical AI" promise, you also need Edge Computing Kits (brains without bodies) or specific robot hardware. 1. The "Digital Twin" Workstation (Required per Student) This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops (MacBooks or non-RTX Windows machines) will not work. GPU (The Bottleneck): NVIDIA RTX 4070 Ti (12GB VRAM) or higher. Why: You need high VRAM to load the USD (Universal Scene Description) assets for the robot and environment, plus run the VLA (Vision-Language-Action) models simultaneously. Ideal: RTX 3090 or 4090 (24GB VRAM) allows for smoother "Sim-to-Real" training. CPU: Intel Core i7 (13th Gen+) or AMD Ryzen 9. Why: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive. RAM: 64 GB DDR5 (32 GB is the absolute minimum, but will crash during complex scene rendering). OS: Ubuntu 22.04 LTS. Note: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience. 2. The "Physical AI" Edge Kit Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the nervous system on a desk before deploying it to a robot. This kit covers Module 3 (Isaac ROS) and Module 4 (VLA). The Brain: NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB). Role: This is the industry standard for embodied AI. Students will deploy their ROS 2 nodes here to understand resource constraints vs. their powerful workstations. The Eyes (Vision): Intel RealSense D435i or D455. Role: Provides RGB (Color) and Depth (Distance) data. Essential for the VSLAM and Perception modules. The Inner Ear (Balance): Generic USB IMU (BNO055) (Often built into the RealSense D435i or Jetson boards, but a separate module helps teach IMU calibration). Voice Interface: A simple USB Microphone/Speaker array (e.g., ReSpeaker) for the "Voice-to-Action" Whisper integration. 3. The Robot Lab For the "Physical" part of the course, you have three tiers of options depending on budget. Option A: The "Proxy" Approach (Recommended for Budget) Use a quadruped (dog) or a robotic arm as a proxy. The software principles (ROS 2, VSLAM, Isaac Sim) transfer 90% effectively to humanoids. Robot: Unitree Go2 Edu (~$1,800 - $3,000). Pros: Highly durable, excellent ROS 2 support, affordable enough to have multiple units. Cons: Not a biped (humanoid). Option B: The "Miniature Humanoid" Approach Small, table-top humanoids. Robot: Unitree H1 is too expensive ($90k+), so look at Unitree G1 (~$16k) or Robotis OP3 (older, but stable, ~$12k). Budget Alternative: Hiwonder TonyPi Pro (~$600). Warning: The cheap kits (Hiwonder) usually run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently. You would use these only for kinematics (walking) and use the Jetson kits for AI. Option C: The "Premium" Lab (Sim-to-Real specific) If the goal is to actually deploy the Capstone to a real humanoid: Robot: Unitree G1 Humanoid. Why: It is one of the few commercially available humanoids that can actually walk dynamically and has an SDK open enough for students to inject their own ROS 2 controllers. 4. Summary of Architecture To teach this successfully, your lab infrastructure should look like this: Component Hardware Function Sim Rig PC with RTX 4080 + Ubuntu 22.04 Runs Isaac Sim, Gazebo, Unity, and trains LLM/VLA models. Edge Brain Jetson Orin Nano Runs the "Inference" stack. Students deploy their code here. Sensors RealSense Camera + Lidar Connected to the Jetson to feed real-world data to the AI. Actuator Unitree Go2 or G1 (Shared) Receives motor commands from the Jetson. If you do not have access to RTX-enabled workstations, we must restructure the course to rely entirely on cloud-based instances (like AWS RoboMaker or NVIDIA's cloud delivery for Omniverse), though this introduces significant latency and cost complexity. Building a "Physical AI" lab is a significant investment. You will have to choose between building a physical On-Premise Lab at Home (High CapEx) versus running a Cloud-Native Lab (High OpEx). Option 2 High OpEx: The "Ether" Lab (Cloud-Native) Best for: Rapid deployment, or students with weak laptops. 1. Cloud Workstations (AWS/Azure) Instead of buying PCs, you rent instances. Instance Type: AWS g5.2xlarge (A10G GPU, 24GB VRAM) or g6e.xlarge. Software: NVIDIA Isaac Sim on Omniverse Cloud (requires specific AMI). Cost Calculation: Instance cost: ~$1.50/hour (spot/on-demand mix). Usage: 10 hours/week × 12 weeks = 120 hours. Storage (EBS volumes for saving environments): ~$25/quarter. Total Cloud Bill: ~$205 per quarter. 2. Local "Bridge" Hardware You cannot eliminate hardware entirely for "Physical AI." You still need the edge devices to deploy the code physically. Edge AI Kits: You still need the Jetson Kit for the physical deployment phase. Cost: $700 (One-time purchase). Robot: You still need one physical robot for the final demo. Cost: $3,000 (Unitree Go2 Standard). The Economy Jetson Student Kit Best for: Learning ROS 2, Basic Computer Vision, and Sim-to-Real control. Component Model Price (Approx.) Notes The Brain NVIDIA Jetson Orin Nano Super Dev Kit (8GB) $249 New official MSRP (Price dropped from ~$499). Capable of 40 TOPS. The Eyes Intel RealSense D435i $349 Includes IMU (essential for SLAM). Do not buy the D435 (non-i). The Ears ReSpeaker USB Mic Array v2.0 $69 Far-field microphone for voice commands (Module 4). Wi-Fi (Included in Dev Kit) $0 The new "Super" kit includes the Wi-Fi module pre-installed. Power/Misc SD Card (128GB) + Jumper Wires $30 High-endurance microSD card required for the OS. TOTAL ~$700 per kit 3. The Latency Trap (Hidden Cost) Simulating in the cloud works well, but controlling a real robot from a cloud instance is dangerous due to latency. Solution: Students train in the Cloud, download the model (weights), and flash it to the local Jetson kit."

## User Scenarios & Testing

### User Story 1 - Create an AI-Native Textbook (Priority: P1)

An author wants to create a textbook using Docusaurus, guided by Spec-Kit Plus and Claude Code, and deploy it to GitHub Pages.

**Why this priority**: This is a core deliverable of the hackathon, enabling the fundamental content creation and publishing process.

**Independent Test**: An author can set up a new Docusaurus project, integrate Spec-Kit Plus, write content, and successfully deploy to GitHub Pages, verifying the basic infrastructure works.

**Acceptance Scenarios**:

1.  **Given** a new project environment, **When** the author initializes a book project with Spec-Kit Plus and Claude Code, **Then** a Docusaurus-based book structure is created and deployable.
2.  **Given** a Docusaurus book project, **When** the author adds content and deploys it, **Then** the book is accessible on GitHub Pages.

---

### User Story 2 - Interact with RAG Chatbot (Priority: P1)

A reader wants to ask questions about the book's content using an embedded Retrieval-Augmented Generation (RAG) chatbot and receive accurate answers, including answers based on selected text.

**Why this priority**: This is the second core deliverable, enhancing reader engagement and learning by providing an interactive Q&A experience.

**Independent Test**: A reader can open the published book, interact with the chatbot, ask questions, and receive contextually relevant answers based on the book's content. Additionally, they can select a portion of text and ask a question specifically about it.

**Acceptance Scenarios**:

1.  **Given** a published book with an embedded RAG chatbot, **When** a reader asks a question about the book content, **Then** the chatbot provides a relevant and accurate answer.
2.  **Given** a published book, **When** a reader selects text in the book and asks a question related to the selected text, **Then** the chatbot uses the selected text as context for its answer.

---

### User Story 3 - Personalized Content (Priority: P2)

A logged-in reader wants to personalize the content of chapters based on their background (software/hardware) by pressing a button at the start of each chapter.

**Why this priority**: This feature offers significant bonus points and adds substantial value by tailoring the learning experience to individual student needs.

**Independent Test**: A user can sign up, provide their software and/or hardware background, log in, navigate to a chapter, and toggle the personalization button to see content dynamically adjusted to their profile.

**Acceptance Scenarios**:

1.  **Given** a logged-in user with a specified background, **When** they click a personalization button in a chapter, **Then** the chapter content dynamically adjusts to their background (e.g., more detailed explanations for beginners, advanced topics for experts).

---

### User Story 4 - Urdu Translation (Priority: P2)

A logged-in reader wants to translate chapter content into Urdu by pressing a button at the start of each chapter.

**Why this priority**: This feature offers significant bonus points and enhances accessibility for a broader, potentially Urdu-speaking, audience.

**Independent Test**: A user can sign up, log in, navigate to a chapter, and toggle a translation button to see the entire chapter content presented in Urdu.

**Acceptance Scenarios**:

1.  **Given** a logged-in user, **When** they click a translation button in a chapter, **Then** the chapter content is translated into Urdu.

---

### User Story 5 - User Authentication (Priority: P3)

A user wants to sign up and sign in to the platform using BetterAuth, providing their software and hardware background during the signup process.

**Why this priority**: This feature is foundational, enabling access to the personalized content and Urdu translation bonus features.

**Independent Test**: A new user can successfully sign up using BetterAuth, providing their background details, and then subsequently log in using their credentials.

**Acceptance Scenarios**:

1.  **Given** a new user, **When** they register via BetterAuth, **Then** their account is created, and their software and hardware background information is successfully captured.
2.  **Given** a registered user, **When** they attempt to log in using their credentials, **Then** they are successfully authenticated and granted access to user-specific features.

---

### Edge Cases

-   What happens when the RAG chatbot cannot find relevant information in the book for a user's query? The chatbot MUST gracefully inform the user that it could not find relevant information within the book's context.
-   How does the system handle an unstable or disconnected internet connection during chatbot interaction or dynamic content loading (personalization/translation)? The system SHOULD provide appropriate feedback to the user and degrade gracefully.
-   What happens if a user tries to access personalized or translated content features without being logged in? The system MUST prompt them to sign up or log in to enable these features.
-   What happens if the translation service (for Urdu) fails or returns an error? The system MUST revert to displaying the original language content and notify the user of the translation failure.

## Requirements

### Functional Requirements

-   **FR-001**: The system MUST create and manage a Docusaurus-based book project structure.
-   **FR-002**: The system MUST enable authors to add, edit, and organize book content within the Docusaurus framework.
-   **FR-003**: The system MUST support deployment of the Docusaurus book to GitHub Pages.
-   **FR-004**: The system MUST integrate an embedded RAG chatbot within the published book interface.
-   **FR-005**: The RAG chatbot MUST accurately answer questions based exclusively on the content of the textbook.
-   **FR-006**: The RAG chatbot MUST be able to answer questions using specific text passages selected by the user as additional context.
-   **FR-007**: The RAG chatbot MUST utilize OpenAI Agents/ChatKit SDKs for agent orchestration, FastAPI for its backend API, Neon Serverless Postgres for data storage, and Qdrant Cloud Free Tier for vector embeddings.
-   **FR-008**: The system MUST implement user authentication (signup and signin) via BetterAuth.com.
-   **FR-009**: During user signup, the system MUST collect information about the user's software and hardware background to enable content personalization.
-   **FR-010**: Logged-in users MUST have the ability to personalize chapter content dynamically based on their recorded background via an interactive element (e.g., a button) at the start of each chapter.
-   **FR-011**: Logged-in users MUST have the ability to translate chapter content into Urdu via an interactive element (e.g., a button) at the start of each chapter.
-   **FR-012**: The project MUST support and incorporate reusable intelligence through Claude Code Subagents and Agent Skills for enhanced development and functionality.

### Key Entities

-   **Book**: The primary entity representing the entire textbook, comprising multiple modules and chapters. Attributes include title, author, deployment status, and content structure.
-   **Chapter**: A sub-entity of a Book, representing a distinct section of educational content. Attributes include title, content text, associated module, and dynamic display properties (personalized/translated).
-   **User**: An individual interacting with the book platform (reader or author). Attributes include authentication status, software background, hardware background, and personalization preferences.
-   **Query**: A question submitted by a User to the RAG Chatbot. Attributes include text content and associated context (e.g., selected book text).
-   **Chatbot Response**: The output generated by the RAG Chatbot in response to a Query. Attributes include answer text and confidence score.

## Success Criteria

### Measurable Outcomes

-   **SC-001**: Authors can successfully deploy updates to the book on GitHub Pages within an average of 5 minutes from initiating the deployment process.
-   **SC-002**: The RAG chatbot achieves a minimum of 90% accuracy in providing relevant and factual answers to questions derived from the book's content, as validated by manual review.
-   **SC-003**: The RAG chatbot responds to 95% of user queries within 3 seconds, measured from query submission to response display.
-   **SC-004**: User signup and signin processes using BetterAuth.com are completed successfully in 99% of attempts, with an average completion time of under 30 seconds.
-   **SC-005**: Personalized content adjustments (e.g., toggling between beginner/expert views) are rendered on-screen within 1 second of user interaction.
-   **SC-006**: Urdu translation of an average-length chapter (approx. 2000 words) is displayed within 3 seconds of the user activating the translation feature.
