#ifndef __COMMAND_PARSER_H__
#define __COMMAND_PARSER_H__

/* 解析控制命令
 * $J:joint,pos,time!  单个关节运动
 * $P:p0,p1,p2,p3,p4,p5,time!  姿态动作
 * $H:time!  关节全部复原
 * $A:joint,angle,time!  单个关节按角度运动
 * $Q:a0,a1,a2,a3,a4,a5,time!  指定角度姿态动作
 */
void command_parser_handle(const char* cmd);

#endif  // __COMMAND_PARSER_H__
