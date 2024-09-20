# Alternate Lunges Rep Counter Assessment

**Cases Covered:**

**1. No Pose Detected:**  
If no pose is detected, the system provides a feedback message to inform the user.  

**2. No Landmarks Detected:**  
The system checks if landmarks are present in the detected pose. If not, it sends a feedback message.  

**3. Insufficient Landmarks:**  
Ensures that a sufficient number of landmarks are detected before proceeding with further calculations.  

**4. Standing Position Detection:**  
The system identifies a standing position and avoids counting minor knee bends as lunges by using a vertical distance threshold between key landmarks.  

**5. Going Down (Lunge Descent Detection):**  
Detects when the user is transitioning from standing to a lunge by analyzing the knee angle and bend in the opposite knee.  

**6. Lunge Bottom Detection:**  
Identifies when the user has reached the bottom of a lunge. This is done by ensuring proper knee angles and measuring the vertical distance between the knees.  

**7. Going Up (Lunge Ascent Detection):**  
Detects the user's transition from the bottom of the lunge back to the standing position.  

**8. Rep Counting:**  
A rep is counted only when the user transitions from the descent to the lunge bottom. The system includes a cooldown mechanism to prevent double counting, ensuring that reps are counted correctly for each leg.  

**9. Progress Calculation and Smoothing:**  
Calculates the progress of each lunge and applies smoothing to handle any fluctuations in the detected landmarks, providing accurate feedback.  

**10. Feedback on Depth Discrepancy:**  
Provides feedback if there is a significant difference in lunge depth between the left and right legs, helping the user maintain balance and symmetry.  

**11. Single-Leg Knee Bends Detection:**  
Detects and avoids counting single-leg knee bends, which can be mistaken for lunges, by analyzing the vertical distance between the knees and the bend in the other knee.  

**12. Holding Lunge Bottom Position:**  
The system ensures that holding the lunge bottom position does not increment the rep counter until the user transitions back to the standing position, preventing premature rep counting.  

