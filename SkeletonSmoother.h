#ifndef SKELETONSMOOTHER_H
#define SKELETONSMOOTHER_H
#include <Kinect.h>
#include <array>

class SkeletonSmoother
{
public:
    static const unsigned int DEPTH_WIDTH = 512;
    static const unsigned int DEPTH_HEIGHT = 424;
    const PointF POINT_ZERO;

    struct JointProp {
        JointProp()
            : type(JointType_Count)
            , isDraw(true)
            , isDirty(true)
        {
            velocity = {0, 0};
            pos = {0, 0};
            attractionPoint = {0, 0};
            drag = {1.5f, 1.5f};
        }

        JointType type;
        PointF velocity, pos, attractionPoint, drag;
        bool isDraw,
             isDirty;//Set to true if the position needs to be updated

        void updatePos(const float &delta, const float &smoothScale = 1)
        {
            velocity.X = (attractionPoint.X - pos.X) * (10 * smoothScale);
            velocity.Y = (attractionPoint.Y - pos.Y) * (10 * smoothScale);

            velocity.X = velocity.X * drag.X;
            velocity.Y = velocity.Y * drag.Y;

            pos.X += velocity.X * delta;
            pos.Y += velocity.Y * delta;
        }

        void reset()
        {
            velocity = {0, 0};
            pos = {0, 0};
            attractionPoint = {0, 0};
            drag = {1.5f, 1.5f};
        }
    };
    typedef std::array<JointProp, JointType_Count> JointArray;

public:
    SkeletonSmoother(ICoordinateMapper *coordinateMapper);
    void updateJointPositions(const unsigned int &bodyIndex, const float &delta, const PointF &screenSize, Joint *joints);
    void reset(const unsigned int &bodyIndex);

    /**
     * @brief The lower the value is the smoother
     * @param scale
     */
    void setSmoothScale(float scale);
    float getSmoothScale() const;

    void setPositionScale(float scale);
    float getPositionScale() const;

    const std::array<JointProp, JointType_Count> &getJointProperties(const unsigned int &bodyIndex) const;
    PointF getJointPosition(const unsigned int &bodyIndex, const unsigned int &type) const;

    void enableJointDrawing(unsigned int bodyIndex, JointType jointType, bool enable);
    bool isJointDrew(unsigned int bodyIndex, JointType jointType) const;

private:
    ICoordinateMapper *m_CoordinateMapper;
    float m_SmoothScale, m_PositionScale;
    std::array<std::array<JointProp, JointType_Count>, BODY_COUNT> m_JointPositions;

private:
    PointF mapBodyPointToScreenPoint(const CameraSpacePoint &bodyPoint, const int &width, const int &height);
    bool pointEquals(const PointF &p1, const PointF &p2);

};

#endif // SKELETONSMOOTHER_H
