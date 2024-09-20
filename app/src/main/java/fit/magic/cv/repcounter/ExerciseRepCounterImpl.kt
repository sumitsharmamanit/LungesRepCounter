// Copyright (c) 2024 Magic Tech Ltd

package fit.magic.cv.repcounter

import fit.magic.cv.PoseLandmarkerHelper
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarkerResult
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark
import kotlin.math.atan2
import kotlin.math.abs

class ExerciseRepCounterImpl : ExerciseRepCounter() {

    private var leftLungeProgress: Float = 0f
    private var rightLungeProgress: Float = 0f
    private var leftLungeState: LungeState = LungeState.STANDING
    private var rightLungeState: LungeState = LungeState.STANDING
    private var lastLeftKneeAngle: Float = 180f
    private var lastRightKneeAngle: Float = 180f
    private var lastRepTime: Long = 0
    private var lastRepLeg: LungeLeg = LungeLeg.NONE

    private val lungeThreshold = 110f
    private val standingThreshold = 165f
    private val repCooldown = 500 // Milliseconds to wait before counting another rep
    private val minOtherKneeBend = 140f
    private val verticalKneeDistanceThreshold = 0.1f

    private enum class LungeState {
        STANDING, GOING_DOWN, LUNGE_BOTTOM, GOING_UP
    }

    private enum class LungeLeg {
        LEFT, RIGHT, NONE
    }

    override fun setResults(resultBundle: PoseLandmarkerHelper.ResultBundle) {
        val poseLandmarkerResults: List<PoseLandmarkerResult> = resultBundle.results

        if (poseLandmarkerResults.isEmpty()) {
            sendFeedbackMessage("No pose detected")
            return
        }

        val firstPoseResult = poseLandmarkerResults[0]
        val poseLandmarks = firstPoseResult.landmarks()

        if (poseLandmarks.isEmpty()) {
            sendFeedbackMessage("No landmarks detected")
            return
        }

        val landmarks = poseLandmarks[0]

        if (landmarks.size < 33) {
            sendFeedbackMessage("Insufficient landmarks detected")
            return
        }

        val leftHip = landmarks[23]
        val leftKnee = landmarks[25]
        val leftAnkle = landmarks[27]

        val rightHip = landmarks[24]
        val rightKnee = landmarks[26]
        val rightAnkle = landmarks[28]

        val leftKneeAngle = calculateAngle(leftHip, leftKnee, leftAnkle)
        val rightKneeAngle = calculateAngle(rightHip, rightKnee, rightAnkle)

        processLunge(leftKneeAngle, rightKneeAngle, isLeft = true, landmarks)
        processLunge(rightKneeAngle, leftKneeAngle, isLeft = false, landmarks)

        updateProgressAndFeedback()

        lastLeftKneeAngle = leftKneeAngle
        lastRightKneeAngle = rightKneeAngle
    }

    private fun processLunge(kneeAngle: Float, otherKneeAngle: Float, isLeft: Boolean, landmarks: List<NormalizedLandmark>) {
        val (progress, state) = if (isLeft) {
            Pair(leftLungeProgress, leftLungeState)
        } else {
            Pair(rightLungeProgress, rightLungeState)
        }

        // Calculate moving average progress based on new progress
        val newProgress = calculateProgress(kneeAngle)
        val smoothedProgress = (progress + newProgress) / 2.0f

        // Get landmark coordinates for knee vertical positions
        val (kneeX, kneeY) = if (isLeft) {
            landmarks[25].x() to landmarks[25].y()
        } else {
            landmarks[26].x() to landmarks[26].y()
        }

        val (otherKneeX, otherKneeY) = if (isLeft) {
            landmarks[26].x() to landmarks[26].y()
        } else {
            landmarks[25].x() to landmarks[25].y()
        }

        val verticalKneeDistance = abs(kneeY - otherKneeY)

        val isOtherKneeBent = otherKneeAngle <= minOtherKneeBend

        // Adjust state transitions based on knee angles and vertical distance
        val newState = when (state) {
            LungeState.STANDING -> {
                if (kneeAngle <= lungeThreshold && isOtherKneeBent && verticalKneeDistance > verticalKneeDistanceThreshold) {
                    LungeState.GOING_DOWN
                } else {
                    LungeState.STANDING
                }
            }
            LungeState.GOING_DOWN -> {
                if (kneeAngle <= lungeThreshold && isOtherKneeBent) {
                    LungeState.LUNGE_BOTTOM
                } else if (kneeAngle > lungeThreshold || !isOtherKneeBent) {
                    LungeState.STANDING
                } else {
                    LungeState.GOING_DOWN
                }
            }
            LungeState.LUNGE_BOTTOM -> {
                if (kneeAngle > lungeThreshold || !isOtherKneeBent) {
                    LungeState.GOING_UP
                } else {
                    LungeState.LUNGE_BOTTOM
                }
            }
            LungeState.GOING_UP -> {
                if (kneeAngle >= standingThreshold) {
                    LungeState.STANDING
                } else {
                    LungeState.GOING_UP
                }
            }
        }

        // Count rep only when transitioning from GOING_DOWN to LUNGE_BOTTOM
        if (state == LungeState.GOING_DOWN && newState == LungeState.LUNGE_BOTTOM) {
            val currentTime = System.currentTimeMillis()
            val currentLeg = if (isLeft) LungeLeg.LEFT else LungeLeg.RIGHT

            if (currentTime - lastRepTime >= repCooldown && lastRepLeg != currentLeg) {
                incrementRepCount()
                sendFeedbackMessage("Good ${if (isLeft) "left" else "right"} lunge!")
                lastRepTime = currentTime
                lastRepLeg = currentLeg
            }
        }

        // Update the progress and state for the corresponding leg
        if (isLeft) {
            leftLungeProgress = smoothedProgress
            leftLungeState = newState
        } else {
            rightLungeProgress = smoothedProgress
            rightLungeState = newState
        }
    }

    private fun calculateProgress(kneeAngle: Float): Float {
        return 1f - ((kneeAngle - lungeThreshold) / (standingThreshold - lungeThreshold)).coerceIn(0f, 1f)
    }

    private fun updateProgressAndFeedback() {
        val overallProgress = maxOf(leftLungeProgress, rightLungeProgress)
        sendProgressUpdate(overallProgress)

        if (abs(leftLungeProgress - rightLungeProgress) > 0.3f) {
            sendFeedbackMessage("Try to keep both legs at similar depths")
        }
    }

    private fun calculateAngle(a: NormalizedLandmark, b: NormalizedLandmark, c: NormalizedLandmark): Float {
        val radians = atan2(c.y() - b.y(), c.x() - b.x()) - atan2(a.y() - b.y(), a.x() - b.x())
        var angle = abs(radians * 180.0 / Math.PI).toFloat()
        if (angle > 180.0f) angle = 360.0f - angle
        return angle
    }
}
