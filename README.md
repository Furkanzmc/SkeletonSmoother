# SkeletonSmoother
This is a class that reduces the jittering on joint positions when they are drawn onto the screen

# Example with Cocos2dx v3.8.1
```C++
SkeletonSmoother m_Smoother(coordinateMapper);
void SkeletonView::update(float delta)
{
//Then inside the update method
  for (int bodyIndex = 0; bodyIndex < BODY_COUNT; bodyIndex++) {
        IBody *body = m_PostureTool.getBody(bodyIndex);
        if (body == nullptr) {
            m_Smoother.reset(bodyIndex);
            return;
        }

        Joint joints[JointType_Count];
        HRESULT hr = body->GetJoints(_countof(joints), joints);
        if (SUCCEEDED(hr)) {
            const PointF screenSize = {m_DrawNode->getContentSize().width, m_DrawNode->getContentSize().height};
            m_Smoother.updateJointPositions(bodyIndex, delta, screenSize, joints);
            drawJoints(bodyIndex);
        }
        else {
            m_Smoother.reset(bodyIndex);
        }
    }
}

void SkeletonView::drawJoints(int bodyIndex)
{
    if (m_IsDrawEnabled) {
        for (unsigned int jointIndex = 0; jointIndex < JointType_Count; jointIndex++) {
            if (m_Smoother.getJointProperties(bodyIndex).at(jointIndex).isDraw) {
                const float circleRadius = (jointIndex == JointType_Head ? 10 : 3) * m_Smoother.getPositionScale();
                const PointF jp = m_Smoother.getJointPosition(bodyIndex, jointIndex);
                m_DrawNode->drawCircle(Vec2(jp.X, jp.Y), circleRadius, 45, 100, true, Color4F::RED);
            }
        }
    }
}
```
