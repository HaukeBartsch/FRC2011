/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.defaultCode;

/**
 *
 * @author Developer
 */
public class Action
{

        public double time;
        public int type;  // -1 = turn counterclockwise +1 = turn clockwise, +2 = forward -2 = reverse
        public double value;
        public static int ACTION_TURN_CCW = -1;
        public static int ACTION_TURN_CW =  +1;
        public static int ACTION_FORWARD =  +2;
        public static int ACTION_BACKWARD = -2;
        public static int ACTION_NONE =      0;

        public Action()
        {
            type = 0;
            value = 0;
        }

        public Action(int type, double value)
        {
            this.type = type;
            if ( type != ACTION_TURN_CW
                 && type != ACTION_TURN_CCW
                 && type != ACTION_FORWARD
                 && type != ACTION_BACKWARD
                 && type != ACTION_NONE)
            {
                System.out.println("Type not set to 0 or 1, type is set to "+this.type);
                this.type = ACTION_FORWARD;
            }
            this.value = value;
        }

        public double returnTime()
        {
            if(type == ACTION_TURN_CW || type == ACTION_TURN_CCW)
            {
                time = (value/90);
                return time;
            }
            else if(type == ACTION_FORWARD || type == ACTION_BACKWARD)
            {
                double distUnit = (66.5*0.5)+0.93;
                time = (value/distUnit);
                return time * .89;
            }
            else
            {
                time = 1;
                return time;
            }
        }

        public int myGetLoopsPerSecond()
        {
            return (GetLoopsPerSec()/200);

        }

        int GetLoopsPerSec()
        {
            return 10000;
        }
}
