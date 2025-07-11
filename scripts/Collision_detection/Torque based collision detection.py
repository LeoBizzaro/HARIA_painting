import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import threading
from collections import deque
import IPython

class RealWorldTorqueCollisionDetector:
    def __init__(self, gui=True):
        """Initialize Franka robot with REAL-WORLD torque-based collision detection"""

        # Connect to PyBullet
        if gui:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)

        # Set up environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1. / 240.)

        # Load ground plane
        self.planeId = p.loadURDF("plane.urdf")

        # Load Franka robot
        self.frankaId = p.loadURDF("franka_panda/panda.urdf",
                                   basePosition=[0, 0, 0],
                                   useFixedBase=True)

        # Get joint information
        self.num_joints = p.getNumJoints(self.frankaId)
        self.joint_indices = []
        self.joint_names = []

        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.frankaId, i)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.joint_indices.append(i)
                self.joint_names.append(joint_info[1].decode('utf-8'))

        print(f"Monitoring joints: {self.joint_names}")

        # IMPROVED TORQUE DETECTION PARAMETERS
        # Much higher base thresholds to avoid false positives during movement
        self.base_torque_thresholds = [15.0, 15.0, 12.0, 12.0, 8.0, 6.0, 4.0]  # Nm - Much higher
        self.dynamic_torque_multiplier = 2.0  # Higher multiplier for movement
        self.velocity_threshold = 0.02  # rad/s - Lower threshold for movement detection

        # Advanced filtering for real-world noise
        self.torque_filter_size = 5  # Increased for better filtering
        self.velocity_filter_size = 3
        self.torque_buffers = [deque(maxlen=self.torque_filter_size) for _ in range(len(self.joint_indices))]
        self.velocity_buffers = [deque(maxlen=self.velocity_filter_size) for _ in range(len(self.joint_indices))]

        # Real-world detection parameters
        self.collision_confirmation_count = 3  # Require more confirmations
        self.detection_history = deque(maxlen=self.collision_confirmation_count)
        self.collision_cooldown_duration = 50  # Longer cooldown
        self.collision_cooldown = 0

        # Baseline torque estimation
        self.baseline_torques = None
        self.baseline_std = None
        self.calibration_samples = []
        self.calibration_complete = False
        self.min_calibration_samples = 30  # More samples for better baseline
        self.calibration_timeout = 60  # Longer timeout
        self.calibration_start_time = None

        # Movement state tracking
        self.movement_state = "stationary"
        self.target_positions = None
        self.movement_start_time = None
        self.movement_grace_period = 2.0  # Grace period at start of movement

        # Real-world collision detection state
        self.collision_detected = False
        self.collision_joint = -1
        self.collision_torque_residual = 0.0
        self.last_collision_time = 0

        # Control flags
        self.running = True
        self.emergency_stop_active = False
        self.simulation_lock = threading.Lock()

        # Add obstacles for testing
        self.obstacles = []
        self.add_realistic_obstacles()

        # Initialize robot
        self.reset_robot()

        print("🔧 IMPROVED TORQUE-BASED COLLISION DETECTION INITIALIZED")
        print("🚨 Higher thresholds to reduce false positives")
        print("📊 Features:")
        print("   • Higher torque thresholds for movement")
        print("   • Movement grace period")
        print("   • Better noise filtering")
        print("   • More confirmation samples required")

    def add_realistic_obstacles(self):
        """Add obstacles positioned for realistic collision testing"""
        obstacle_positions = [
            [0.4, 0.15, 0.2],  # Reachable obstacle
            [0.35, -0.2, 0.25],  # Side obstacle
            [0.45, 0.0, 0.3],  # Front obstacle
            [0.3, 0.3, 0.15],  # Corner obstacle
        ]

        colors = [
            [0.8, 0.2, 0.2, 1.0],  # Red
            [0.2, 0.8, 0.2, 1.0],  # Green
            [0.2, 0.2, 0.8, 1.0],  # Blue
            [0.8, 0.8, 0.2, 1.0],  # Yellow
        ]

        for i, (pos, color) in enumerate(zip(obstacle_positions, colors)):
            # Create realistic obstacles
            box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06])
            box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.06, 0.06, 0.06],
                                             rgbaColor=color)

            obstacle_id = p.createMultiBody(baseMass=1.0,
                                            baseCollisionShapeIndex=box_collision,
                                            baseVisualShapeIndex=box_visual,
                                            basePosition=pos)

            self.obstacles.append({
                'id': obstacle_id,
                'position': pos,
                'name': f"Obstacle_{i + 1}_{['Red', 'Green', 'Blue', 'Yellow'][i]}"
            })

            print(f"Added {self.obstacles[-1]['name']} at {pos}")

    def reset_robot(self):
        """Reset robot to neutral position"""
        neutral_positions = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

        with self.simulation_lock:
            for i, pos in enumerate(neutral_positions):
                if i < len(self.joint_indices):
                    p.resetJointState(self.frankaId, self.joint_indices[i], pos)

        # Reset collision detection state
        self.collision_detected = False
        self.collision_joint = -1
        self.emergency_stop_active = False
        self.collision_cooldown = 0
        self.movement_state = "stationary"

        # Clear buffers
        for buffer in self.torque_buffers:
            buffer.clear()
        for buffer in self.velocity_buffers:
            buffer.clear()
        self.detection_history.clear()

        # Stabilize robot
        print("🔄 Stabilizing robot...")
        for step in range(120):
            if not p.isConnected():
                break
            with self.simulation_lock:
                for i, pos in enumerate(neutral_positions):
                    if i < len(self.joint_indices):
                        p.setJointMotorControl2(self.frankaId, self.joint_indices[i],
                                                p.POSITION_CONTROL,
                                                targetPosition=pos,
                                                force=80)
                p.stepSimulation()
            time.sleep(1. / 240.)

        print("✅ Robot reset to neutral position")
        self.start_calibration()

    def start_calibration(self):
        """Start calibration procedure"""
        print("🔧 Starting improved torque calibration...")
        print("   Collecting baseline torque data...")

        self.calibration_samples = []
        self.calibration_complete = False
        self.calibration_start_time = time.time()

        # Reset baseline
        self.baseline_torques = None
        self.baseline_std = None

    def update_calibration(self, joint_states):
        """Update calibration with new samples"""
        if self.calibration_complete:
            return

        # Check timeout
        if time.time() - self.calibration_start_time > self.calibration_timeout:
            print("⚠️  Calibration timeout - using current samples")
            if len(self.calibration_samples) >= 10:
                self.finalize_calibration()
            return

        # Extract torques and velocities
        torques = np.array([state[3] for state in joint_states])
        velocities = np.array([state[1] for state in joint_states])

        # Strict stability requirement for calibration
        if np.max(np.abs(velocities)) < 0.005:  # Very strict for calibration
            filtered_torques = self.apply_torque_filter(torques)
            self.calibration_samples.append(filtered_torques)

            # Show progress
            if len(self.calibration_samples) % 10 == 0:
                print(
                    f"   📊 Calibration progress: {len(self.calibration_samples)}/{self.min_calibration_samples} samples")

            if len(self.calibration_samples) >= self.min_calibration_samples:
                self.finalize_calibration()

    def finalize_calibration(self):
        """Finalize calibration with improved statistics"""
        if len(self.calibration_samples) < 10:
            return

        samples_array = np.array(self.calibration_samples)

        # Use robust statistics
        self.baseline_torques = np.median(samples_array, axis=0)
        self.baseline_std = np.std(samples_array, axis=0)

        # Ensure reasonable minimum standard deviation
        self.baseline_std = np.maximum(self.baseline_std, 0.2)

        self.calibration_complete = True

        print(f"✅ Calibration complete with {len(self.calibration_samples)} samples")
        print(f"   Baseline torques: {[round(t, 3) for t in self.baseline_torques]} Nm")
        print(f"   Standard deviations: {[round(s, 3) for s in self.baseline_std]} Nm")
        print("🚀 Ready for collision detection!")

    def apply_torque_filter(self, raw_torques):
        """Apply advanced filtering for real-world noise"""
        filtered_torques = []

        for i, torque in enumerate(raw_torques):
            self.torque_buffers[i].append(torque)

            if len(self.torque_buffers[i]) >= 3:
                # Use median filter for outlier rejection
                buffer_array = np.array(list(self.torque_buffers[i]))
                filtered_torques.append(np.median(buffer_array))
            else:
                filtered_torques.append(torque)

        return np.array(filtered_torques)

    def apply_velocity_filter(self, raw_velocities):
        """Apply velocity filtering"""
        filtered_velocities = []

        for i, velocity in enumerate(raw_velocities):
            self.velocity_buffers[i].append(velocity)

            if len(self.velocity_buffers[i]) >= 2:
                # Moving average for velocities
                buffer_array = np.array(list(self.velocity_buffers[i]))
                filtered_velocities.append(np.mean(buffer_array))
            else:
                filtered_velocities.append(velocity)

        return np.array(filtered_velocities)

    def detect_collision_real_world(self):
        """IMPROVED torque-based collision detection"""
        if not p.isConnected():
            return None

        # Skip detection during cooldown
        if self.collision_cooldown > 0:
            self.collision_cooldown -= 1
            return {
                'collision_detected': False,
                'collision_joint': -1,
                'collision_type': 'cooldown',
                'torque_residual': 0,
                'confidence': 0
            }

        # Get current joint states
        joint_states = []
        for joint_idx in self.joint_indices:
            try:
                state = p.getJointState(self.frankaId, joint_idx)
                joint_states.append(state)
            except p.error:
                return None

        # Update calibration if needed
        if not self.calibration_complete:
            self.update_calibration(joint_states)
            return {
                'collision_detected': False,
                'collision_joint': -1,
                'collision_type': 'calibrating',
                'torque_residual': 0,
                'confidence': 0,
                'calibration_progress': len(self.calibration_samples)
            }

        # Extract and filter data
        raw_torques = np.array([state[3] for state in joint_states])
        raw_velocities = np.array([state[1] for state in joint_states])

        filtered_torques = self.apply_torque_filter(raw_torques)
        filtered_velocities = self.apply_velocity_filter(raw_velocities)

        # Determine movement state
        max_velocity = np.max(np.abs(filtered_velocities))
        if max_velocity < self.velocity_threshold:
            self.movement_state = "stationary"
        else:
            self.movement_state = "moving"

        # GRACE PERIOD: Skip detection at start of movement
        if (self.movement_start_time is not None and
                time.time() - self.movement_start_time < self.movement_grace_period):
            return {
                'collision_detected': False,
                'collision_joint': -1,
                'collision_type': 'grace_period',
                'torque_residual': 0,
                'confidence': 0
            }

        # Calculate torque residuals
        torque_residuals = np.abs(filtered_torques - self.baseline_torques)

        # IMPROVED adaptive thresholding
        adaptive_thresholds = []
        for i, (base_threshold, std_dev) in enumerate(zip(self.base_torque_thresholds, self.baseline_std)):
            # Start with base threshold
            threshold = base_threshold

            # Significantly increase threshold during movement
            if self.movement_state == "moving":
                threshold *= self.dynamic_torque_multiplier

            # Add noise-based margin
            threshold += 3.0 * std_dev  # Larger margin

            # Joint-specific adjustments (some joints naturally have higher torques)
            if i in [1, 3]:  # Joints that typically have higher torques
                threshold *= 1.5

            adaptive_thresholds.append(threshold)

        # Detect collisions
        collision_detected = False
        collision_joint = -1
        max_residual = 0

        for i, (residual, threshold) in enumerate(zip(torque_residuals, adaptive_thresholds)):
            if residual > threshold:
                collision_detected = True
                if residual > max_residual:
                    max_residual = residual
                    collision_joint = i

        # Strict confirmation logic
        self.detection_history.append(collision_detected)

        if len(self.detection_history) >= self.collision_confirmation_count:
            recent_detections = list(self.detection_history)[-self.collision_confirmation_count:]
            confirmed_collision = sum(recent_detections) == self.collision_confirmation_count  # ALL must be true

            if confirmed_collision:
                # Calculate confidence
                confidence = min(max_residual / adaptive_thresholds[collision_joint], 3.0) / 3.0

                # Activate cooldown
                self.collision_cooldown = self.collision_cooldown_duration

                return {
                    'collision_detected': True,
                    'collision_joint': collision_joint,
                    'collision_type': 'torque_confirmed',
                    'torque_residual': max_residual,
                    'threshold': adaptive_thresholds[collision_joint],
                    'confidence': confidence,
                    'movement_state': self.movement_state,
                    'measured_torque': filtered_torques[collision_joint],
                    'baseline_torque': self.baseline_torques[collision_joint]
                }

        return {
            'collision_detected': False,
            'collision_joint': -1,
            'collision_type': 'none',
            'torque_residual': max_residual,
            'confidence': 0,
            'movement_state': self.movement_state
        }

    def move_with_real_world_monitoring(self, target_pos, duration=8.0):
        """Move with improved collision monitoring"""
        if self.emergency_stop_active:
            print("❌ Emergency stop active! Reset robot first.")
            return False

        if not self.calibration_complete:
            print("❌ Calibration not complete! Please wait for calibration.")
            print(f"   📊 Current progress: {len(self.calibration_samples)}/{self.min_calibration_samples} samples")
            return False

        # Get current joint positions
        current_joint_positions = []
        for joint_idx in self.joint_indices:
            try:
                current_joint_positions.append(p.getJointState(self.frankaId, joint_idx)[0])
            except p.error:
                return False

        # Calculate target joint angles
        target_joint_angles = self.cartesian_to_joint_angles(target_pos)
        if target_joint_angles is None:
            print("❌ Could not solve inverse kinematics")
            return False

        current_joint_positions = np.array(current_joint_positions)
        target_joint_angles = np.array(target_joint_angles)

        print(f"🚀 Starting IMPROVED movement with torque monitoring...")
        print(f"   Target position: {[round(p, 3) for p in target_pos]}")
        print(f"   Grace period: {self.movement_grace_period}s")

        start_time = time.time()
        self.movement_start_time = start_time
        collision_detected = False

        # Movement loop
        while time.time() - start_time < duration:
            if not p.isConnected():
                return False

            # Collision detection
            collision_info = self.detect_collision_real_world()
            if collision_info and collision_info['collision_detected']:
                self.handle_real_world_collision(collision_info)
                collision_detected = True
                break

            # Progress calculation
            progress = (time.time() - start_time) / duration

            # Smoother S-curve for movement
            if progress < 0.3:
                s = 0.5 * (progress / 0.3) ** 2
            elif progress < 0.7:
                s = 0.5 + 2.5 * (progress - 0.3)
            else:
                s = 1.0 - 0.5 * ((1 - progress) / 0.3) ** 2

            s = min(s, 1.0)

            interpolated_positions = current_joint_positions + s * (target_joint_angles - current_joint_positions)

            # Apply control
            with self.simulation_lock:
                for i, pos in enumerate(interpolated_positions):
                    if i < len(self.joint_indices):
                        try:
                            p.setJointMotorControl2(self.frankaId,
                                                    self.joint_indices[i],
                                                    p.POSITION_CONTROL,
                                                    targetPosition=pos,
                                                    force=150,  # Increased force for better movement
                                                    maxVelocity=0.8)  # Increased max velocity
                        except p.error:
                            return False

                p.stepSimulation()

            # Show progress every 2 seconds
            if int(time.time() - start_time) % 2 == 0 and (time.time() - start_time) % 1 < 0.1:
                current_pos = self.get_end_effector_position()
                if current_pos:
                    print(f"   📍 Progress: {int(progress * 100)}% - Position: {[round(p, 3) for p in current_pos]}")

            time.sleep(1. / 240.)

        # Clear movement tracking
        self.movement_start_time = None

        if not collision_detected:
            print("✅ Movement completed - no collision detected")
            final_pos = self.get_end_effector_position()
            if final_pos:
                print(f"   📍 Final position: {[round(p, 3) for p in final_pos]}")

        return collision_detected

    def handle_real_world_collision(self, collision_info):
        """Handle collision with detailed information"""
        joint_name = self.joint_names[collision_info['collision_joint']]

        print(f"\n🚨 REAL-WORLD COLLISION DETECTED!")
        print(f"   🦾 Joint: {joint_name} (Joint {collision_info['collision_joint']})")
        print(f"   📊 Torque residual: {collision_info['torque_residual']:.3f} Nm")
        print(f"   📏 Threshold: {collision_info['threshold']:.3f} Nm")
        print(f"   🎯 Confidence: {collision_info['confidence']:.2f}")
        print(f"   🏃 Movement state: {collision_info['movement_state']}")
        print(f"   📈 Measured torque: {collision_info['measured_torque']:.3f} Nm")
        print(f"   📐 Baseline torque: {collision_info['baseline_torque']:.3f} Nm")

        self.emergency_stop()

    def emergency_stop(self):
        """Emergency stop procedure"""
        print("🛑 EMERGENCY STOP ACTIVATED")
        self.emergency_stop_active = True

        # Hold current positions
        with self.simulation_lock:
            for joint_idx in self.joint_indices:
                try:
                    current_pos = p.getJointState(self.frankaId, joint_idx)[0]
                    p.setJointMotorControl2(self.frankaId, joint_idx,
                                            p.POSITION_CONTROL,
                                            targetPosition=current_pos,
                                            force=100)
                except p.error:
                    break

        print("💡 Use reset() to recover from emergency stop")

    def cartesian_to_joint_angles(self, target_pos):
        """Calculate joint angles for target position"""
        try:
            joint_angles = p.calculateInverseKinematics(
                self.frankaId,
                11,  # End-effector link
                target_pos,
                maxNumIterations=1000,
                residualThreshold=0.01
            )
            return joint_angles[:len(self.joint_indices)]
        except:
            return None

    def get_end_effector_position(self):
        """Get current end-effector position"""
        try:
            link_state = p.getLinkState(self.frankaId, 11)
            return link_state[0]
        except:
            return None

    def start_simulation_loop(self):
        """Start simulation loop"""

        def simulation_loop():
            while self.running:
                if p.isConnected():
                    try:
                        # Handle calibration
                        if not self.calibration_complete:
                            joint_states = []
                            for joint_idx in self.joint_indices:
                                try:
                                    state = p.getJointState(self.frankaId, joint_idx)
                                    joint_states.append(state)
                                except:
                                    break
                            if joint_states:
                                self.update_calibration(joint_states)

                        # Step simulation only when stationary
                        if self.movement_state == "stationary":
                            with self.simulation_lock:
                                p.stepSimulation()


                        time.sleep(1. / 240.)
                    except:
                        break
                else:
                    break

        self.sim_thread = threading.Thread(target=simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    # User interface methods
    def move_to_obstacle(self, index):
        """Move toward obstacle"""
        if index < 0 or index >= len(self.obstacles):
            print(f"❌ Invalid obstacle index. Use 0-{len(self.obstacles) - 1}")
            return

        obstacle = self.obstacles[index]
        print(f"🎯 Moving to {obstacle['name']} at {obstacle['position']}")

        # Move close to obstacle
        target_pos = obstacle['position'].copy()
        target_pos[2] += 0.02  # Slightly above for approach

        collision_detected = self.move_with_real_world_monitoring(target_pos, duration=10.0)

        if collision_detected:
            print(f"✅ Collision detected with {obstacle['name']}!")
        else:
            print(f"❌ No collision detected with {obstacle['name']}")

    def move_through_obstacle(self, index):
        """Move through obstacle to guarantee collision"""
        if index < 0 or index >= len(self.obstacles):
            print(f"❌ Invalid obstacle index. Use 0-{len(self.obstacles) - 1}")
            return

        obstacle = self.obstacles[index]
        print(f"🎯 Moving THROUGH {obstacle['name']} - guaranteed collision test")

        # Get current position
        current_pos = self.get_end_effector_position()
        if current_pos is None:
            print("❌ Could not get current position")
            return

        # Calculate target beyond obstacle
        obstacle_pos = np.array(obstacle['position'])
        current_pos = np.array(current_pos)

        direction = obstacle_pos - current_pos
        direction = direction / np.linalg.norm(direction)

        # Target position beyond obstacle
        target_pos = obstacle_pos + direction * 0.15

        print(f"📍 Path: {[round(p, 3) for p in current_pos]} → {[round(p, 3) for p in target_pos]}")

        collision_detected = self.move_with_real_world_monitoring(target_pos, duration=12.0)

        if collision_detected:
            print(f"✅ Successfully detected collision with {obstacle['name']}!")
        else:
            print(f"❌ Unexpected: No collision detected")

    def list_obstacles(self):
        """List all obstacles"""
        print("\n📋 Available obstacles:")
        for i, obstacle in enumerate(self.obstacles):
            print(f"  {i}: {obstacle['name']} at {obstacle['position']}")
        print()

    def show_status(self):
        """Show system status"""
        print("\n📊 SYSTEM STATUS:")
        print(f"   🔧 Calibration complete: {self.calibration_complete}")
        if not self.calibration_complete:
            print(f"   📊 Calibration progress: {len(self.calibration_samples)}/{self.min_calibration_samples} samples")
        else:
            print(f"   📊 Baseline torques: {[round(t, 3) for t in self.baseline_torques]}")
        print(f"   🚨 Emergency stop: {self.emergency_stop_active}")
        print(f"   🏃 Movement state: {self.movement_state}")
        print(f"   ⏱️ Collision cooldown: {self.collision_cooldown}")

        # Current position
        pos = self.get_end_effector_position()
        if pos:
            print(f"   📍 Current position: {[round(p, 3) for p in pos]}")

    def cleanup(self):
        """Clean up simulation"""
        self.running = False
        if hasattr(self, 'sim_thread'):
            self.sim_thread.join(timeout=1.0)

        try:
            if p.isConnected():
                p.disconnect()
                print("🧹 Cleaned up PyBullet simulation")
        except:
            pass


def main():
    """Main function with interactive console"""
    print("=" * 70)
    print("🤖 IMPROVED TORQUE-BASED COLLISION DETECTION")
    print("=" * 70)
    print("🚨 This system uses ONLY torque analysis - no PyBullet contacts!")
    print("💡 Higher thresholds to reduce false positives")
    print("🎮 Robot will now move visually before collision detection!")

    # Create detector
    detector = RealWorldTorqueCollisionDetector(gui=True)

    # Start simulation
    detector.start_simulation_loop()

    print("\n🎮 Interactive Console Ready!")
    print("⚠️  Calibration will complete automatically in ~30-60 seconds")

    try:
        while True:
            try:
                command = input("\n>> ").strip()

                if not command:
                    continue

                if command == 'help':
                    print("\n📋 Available Commands:")
                    print("  move_to_obstacle(index)      - Move to obstacle (0-3)")
                    print("  move_through_obstacle(index) - Move through obstacle (collision test)")
                    print("  reset()                      - Reset robot and recalibrate")
                    print("  status()                     - Show system status")
                    print("  list_obstacles()             - Show all obstacles")
                    print("  current_pos()                - Show current position")
                    print("  exit                         - Exit program")

                elif command == 'exit':
                    break

                elif command == 'reset()':
                    detector.reset_robot()

                elif command == 'status()':
                    detector.show_status()

                elif command == 'list_obstacles()':
                    detector.list_obstacles()

                elif command == 'embed':
                    IPython.embed()

                elif command == 'current_pos()':
                    pos = detector.get_end_effector_position()
                    if pos:
                        print(f"Current position: {[round(p, 3) for p in pos]}")

                elif command.startswith('move_to_obstacle('):
                    try:
                        index = int(command.split('(')[1].split(')')[0])
                        detector.move_to_obstacle(index)
                    except:
                        print("❌ Invalid format. Use: move_to_obstacle(0)")

                elif command.startswith('move_through_obstacle('):
                    try:
                        index = int(command.split('(')[1].split(')')[0])
                        detector.move_through_obstacle(index)
                    except:
                        print("❌ Invalid format. Use: move_through_obstacle(0)")



                else:

                    print("❌ Unknown command. Type 'help' for available commands.")


            except KeyboardInterrupt:

                print("\n🛑 Interrupted by user")

                break

            except Exception as e:

                print(f"❌ Error: {e}")


    except KeyboardInterrupt:

        print("\n🛑 Shutting down...")

    finally:

        print("🧹 Cleaning up...")

        detector.cleanup()

        print("👋 Goodbye!")


if __name__ == "__main__":
    main()