#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
话题列表节点
用于获取并打印当前可用的话题（包括相机相关话题）
"""

import rospy


def main():
    rospy.init_node('topic_list_node', anonymous=True)
    rospy.loginfo("=" * 60)
    rospy.loginfo("当前 ROS 话题列表:")
    rospy.loginfo("=" * 60)

    topics = rospy.get_published_topics()
    for topic, msg_type in topics:
        rospy.loginfo("%s  (%s)", topic, msg_type)

    rospy.loginfo("=" * 60)
    rospy.loginfo("话题列表获取完成")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
