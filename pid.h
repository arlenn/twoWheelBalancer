/*
 * pid.h
 *
 *  Created on: Apr 20, 2018
 *      Author: hardyn
 */

#ifndef PID_H_
#define PID_H_

int32_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double kc, double failAngle, double maxAng, double deltaTime, double minPercent);

void posPid (double setPoint, double relDistance, double kp, double ki, double kd, double kc);

#endif /* PID_H_ */
