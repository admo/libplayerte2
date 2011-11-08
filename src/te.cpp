#include "te.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <algorithm>

#include <unistd.h>

#ifndef SIGN
#define SIGN(x) (((x) == 0) ? 0 : (((x) > 0) ? 1 : -1))
#endif

extern PlayerTime *GlobalTime;

// Initialization function

Driver* TE_Init(ConfigFile* cf, int section) {
    return ((Driver*) (new TE(cf, section)));
}

// a driver registration function

void TE_Register(DriverTable* table) {
    char nazwa[] = "te";
    table->AddDriver(nazwa, TE_Init);
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor

TE::TE(ConfigFile* cf, int section)
: Driver(cf, section, false, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_POSITION2D_CODE) {
    dist_eps = cf->ReadTupleLength(section, "goal_tol", 0, 0.5);
    ang_eps = cf->ReadTupleAngle(section, "goal_tol", 1, DTOR(10.0));

    vx_max = cf->ReadFloat(section, "max_speed", 0.3);
    va_max = cf->ReadFloat(section, "max_rot", DTOR(45.0));
    vx_min = cf->ReadFloat(section, "min_speed", 0.05);
    va_min = cf->ReadFloat(section, "min_rot", DTOR(10.0));

    min_dist = cf->ReadLength(section, "min_dist", 0.3);
    warning_dist_on = cf->ReadLength(section, "warning_dist_on", 0.4);
    warning_dist_off = cf->ReadLength(section, "warning_dist_off", 0.41);
    obs_dist = cf->ReadLength(section, "obs_dist", 0.7);

    laser_min_angle = cf->ReadFloat(section, "laser_min_angle",
            -DTOR(90));
    laser_max_angle = cf->ReadFloat(section, "laser_max_angle",
            DTOR(90));



    k_a = cf->ReadFloat(section, "k_a", 1);

    odom = NULL;
    if (cf->ReadDeviceAddr(&odom_addr, section, "requires",
            PLAYER_POSITION2D_CODE, -1, "output") != 0) {
        SetError(-1);
        return;
    }

    localize = NULL;
    if (cf->ReadDeviceAddr(&localize_addr, section, "requires",
            PLAYER_POSITION2D_CODE, -1, "input") != 0) {
        SetError(-1);
        return;
    }

    laser = NULL;
    memset(&laser_addr, 0, sizeof (player_devaddr_t));
    cf->ReadDeviceAddr(&laser_addr, section, "requires",
            PLAYER_LASER_CODE, -1, NULL);
    if (laser_addr.interf) {
        laser_buffer = cf->ReadInt(section, "laser_buffer", 10);
    }

    return;
}

TE::~TE() {
    return;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device (called by server thread).

int TE::Setup() {
    // Initialise the underlying position device.
    if (SetupOdom() != 0)
        return -1;

    active_goal = false;

    // Initialise the laser.
    if (laser_addr.interf && SetupLaser() != 0)
        return -1;

    stall = false;
    turning_in_place = false;
    last_odom_pose.px = last_odom_pose.py = last_odom_pose.pa = std::numeric_limits<float>::max();

    obstacle = false;

    waiting = false;

    // Start the driver thread.
    StartThread();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device (called by server thread).

int TE::Shutdown() {
    // Stop the driver thread.
    StopThread();

    // Stop the laser
    if (laser)
        ShutdownLaser();

    // Stop the odom device.
    ShutdownOdom();

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the underlying odom device.

int TE::SetupOdom() {
    // Setup the output position device
    if (!(odom = deviceTable->GetDevice(odom_addr))) {
        PLAYER_ERROR("TE::SetupOdom Unable to locate suitable output position device");
        return -1;
    }
    if (odom->Subscribe(InQueue) != 0) {
        PLAYER_ERROR("TE::SetupOdom Unable to subscribe to output position device");
        return -1;
    }

    // Setup the input position device
    if (!(localize = deviceTable->GetDevice(localize_addr))) {
        PLAYER_ERROR("TE::SetupOdom Unable to locate suitable input position device");
        return -1;
    }

    if (localize->Subscribe(InQueue) != 0) {
        PLAYER_ERROR("TE::SetupOdom Unable to subscribe to input position device");
        return -1;
    }

    // Get the odometry geometry
    Message* msg = 0;
    if (!(msg = odom->Request(InQueue, PLAYER_MSGTYPE_REQ, PLAYER_POSITION2D_REQ_GET_GEOM, NULL, 0, NULL, false))
            || (msg->GetHeader()->size != sizeof (player_position2d_geom_t))) {
        PLAYER_ERROR("TE::SetupOdom Failed to get geometry of underlying position device");
        if (msg)
            delete msg;
        return (-1);
    }
    player_position2d_geom_t* geom = (player_position2d_geom_t*) msg->GetPayload();

    robot_geom = *geom;
    PLAYER_MSG5(0, "TE::SetupOdom Robot geom: %.3f %.3f %.3f %.3f %.3f",
            robot_geom.size.sl,
            robot_geom.size.sw,
            robot_geom.pose.px,
            robot_geom.pose.py,
            RTOD(robot_geom.pose.pa));

    delete msg;

    memset(&odom_pose, 0, sizeof (player_pose_t));
    memset(&odom_vel, 0, sizeof (player_pose_t));
    odom_stall = false;

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the underlying odom device.

int TE::ShutdownOdom() {
    // Stop the robot before unsubscribing
    PutPositionCmd(0.0, 0.0);
    odom->Unsubscribe(InQueue);
    localize->Unsubscribe(InQueue);
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Set up the laser

int TE::SetupLaser() {
    if (!(laser = deviceTable->GetDevice(laser_addr))) {
        PLAYER_ERROR("TE::SetupLaser Unable to locate suitable laser device");
        return -1;
    }
    if (laser->Subscribe(InQueue) != 0) {
        PLAYER_ERROR("TE::SetupLaser Unable to subscribe to laser device");
        return -1;
    }

    player_laser_geom_t* cfg;
    Message* msg;

    // Get the laser pose
    if (!(msg = laser->Request(InQueue,
            PLAYER_MSGTYPE_REQ,
            PLAYER_LASER_REQ_GET_GEOM,
            NULL, 0, NULL, false))) {
        PLAYER_ERROR("TE::SetupLaser Failed to get laser geometry");
        return (-1);
    }

    // Store the laser pose
    cfg = (player_laser_geom_t*) msg->GetPayload();
    laser_pose = cfg->pose;
    delete msg;

    num_laser_scans = 0;

    return 0;
}


////////////////////////////////////////////////////////////////////////////////
// Shut down the laser

int TE::ShutdownLaser() {
    laser->Unsubscribe(InQueue);
    return 0;
}


// Send a command to the motors

void
TE::PutPositionCmd(double vx, double va) {
    player_position2d_cmd_vel_t cmd;

    memset(&cmd, 0, sizeof (player_position2d_cmd_vel_t));

    cmd.vel.px = vx;
    cmd.vel.pa = va;
    cmd.state = 1;

    odom->PutMsg(InQueue,
            PLAYER_MSGTYPE_CMD,
            PLAYER_POSITION2D_CMD_VEL,
            (void*) &cmd, sizeof (cmd), NULL);
}

void
TE::ProcessInputOdom(player_msghdr_t* hdr, player_position2d_data_t* data) {
    odom_pose = data->pos;

    player_msghdr_t newhdr = *hdr;
    newhdr.addr = device_addr;
    player_position2d_data_t newdata;

    newdata.pos = data->pos;
    newdata.vel = odom_vel;
    if (data->stall) {
        PutPositionCmd(0.0, 0.0);
        waiting = true;
        newdata.stall = 0;
    } else {
        newdata.stall = 0;
        waiting = false;
    }

    // stall indicates that we're stuck (either TE threw an emergency 
    // stop or it was failing to make progress).  Set the stall flag to let
    // whoever's listening that we've given up.
    if (stall)
        newdata.stall = 1;

    Publish(NULL, &newhdr, &newdata);
}

void
TE::ProcessOutputOdom(player_msghdr_t*, player_position2d_data_t* data) {
    odom_vel = data->vel;
    // Stage's stall flag seems to be broken
    //odom_stall = data->stall;
    odom_stall = false;
}

void
TE::ProcessLaser(player_msghdr_t*, player_laser_data_t* scan) {
    double r, b, db;
    double dmin = obs_dist;
    obstacle = false;
    beta = M_PI / 2;

    db = scan->resolution;

    for (unsigned int i = 0; i < scan->ranges_count; i++) {
        b = scan->min_angle + (i * db);
        r = scan->ranges[i];
        if (b >= laser_min_angle && b <= laser_max_angle) {
            if (r < dmin) {
                obstacle = true;
                dmin = r;
                beta = b;
            }
        }
    }
    nearObstDist = dmin;
    if (dmin <= min_dist) {
        PLAYER_MSG0(0, "TE::ProcessLaser Stoped");
        waiting = true;
        PutPositionCmd(0, 0);
        stall = true;
    }
}

void
TE::ProcessCommand(player_msghdr_t*, player_position2d_cmd_vel_t* cmd) {
    if (!cmd->state) {
        PutPositionCmd(0.0, 0.0);
        active_goal = false;
    } else {
        PLAYER_MSG2(2, "TE::ProcessCommand Stopped by velocity command (%.3f %.3f)",
                cmd->vel.px, RTOD(cmd->vel.pa));
        // TODO: bylo wylaczone w ostatniej wersji, ale wtedy nie zatrzymuje sie
        PutPositionCmd(cmd->vel.px, cmd->vel.pa);
        active_goal = false;
    }
    if (cmd->vel.px < 0)
        dir = -1;
    else
        dir = 1;
}

void
TE::ProcessCommand(player_msghdr_t*, player_position2d_cmd_pos_t* cmd) {
    PLAYER_MSG3(2, "TE::ProcessCommand New goal: (%.3f %.3f %.3f)",
            cmd->pos.px,
            cmd->pos.py,
            RTOD(cmd->pos.pa));
    // position control;  cache the goal and we'll process it in the main
    // loop.  also cache vel.px, which tells us whether to go forward or
    // backward to get to the goal.
    goal = cmd->pos;

    active_goal = true;
    turning_in_place = false;
    stall = false;
    GlobalTime->GetTimeDouble(&translate_start_time);
    last_odom_pose = odom_pose;
}

////////////////////////////////////////////////////////////////////////////////
// Process an incoming message

int TE::ProcessMessage(MessageQueue* resp_queue,
        player_msghdr * hdr,
        void * data) {
    // Is it new odometry data?
    if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
            PLAYER_POSITION2D_DATA_STATE,
            odom_addr)) {
        ProcessOutputOdom(hdr, (player_position2d_data_t*) data);

        // In case the input and output are the same device
        if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
                PLAYER_POSITION2D_DATA_STATE,
                localize_addr))
            ProcessInputOdom(hdr, (player_position2d_data_t*) data);

        return (0);
    }// Is it new localization data?
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
            PLAYER_POSITION2D_DATA_STATE,
            localize_addr)) {
        ProcessInputOdom(hdr, (player_position2d_data_t*) data);
        return (0);
    }// Is it a new laser scan?
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,
            PLAYER_LASER_DATA_SCAN,
            laser_addr)) {
        ProcessLaser(hdr, (player_laser_data_t*) data);
        return (0);
    }// Is it a new goal?
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
            PLAYER_POSITION2D_CMD_POS,
            device_addr)) {
        ProcessCommand(hdr, (player_position2d_cmd_pos_t*) data);
        return 0;
    } else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,
            PLAYER_POSITION2D_CMD_VEL,
            device_addr)) {
        ProcessCommand(hdr, (player_position2d_cmd_vel_t*) data);
        return 0;
    }// Is it a request for the underlying device?
    else if (Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ, -1, device_addr)) {
        // Pass the request on to the underlying position device and wait for
        // the reply.
        Message* msg;

        if (!(msg = odom->Request(InQueue,
                hdr->type,
                hdr->subtype,
                (void*) data,
                hdr->size,
                &hdr->timestamp))) {
            PLAYER_WARN1("TE::ProcessCommand Failed to forward config request with subtype: %d\n",
                    hdr->subtype);
            return (-1);
        }

        player_msghdr_t* rephdr = msg->GetHeader();
        void* repdata = msg->GetPayload();
        // Copy in our address and forward the response
        rephdr->addr = device_addr;
        Publish(resp_queue, rephdr, repdata);
        delete msg;
        return (0);
    } else
        return -1;
}

double TE::Threshold(double v, double v_min, double v_max) {
    if (v == 0.0)
        return (v);
    else if (v > 0.0) {
        v = std::min(v, v_max);
        v = std::max(v, v_min);
        return (v);
    } else {
        v = std::max(v, -v_max);
        v = std::min(v, -v_min);
        return (v);
    }
}

void
TE::Main() {
    double g_dx, g_da;
    double alpha, theta, phi;
    double vx, va;
    player_pose_t relativeGoal;
    player_pose_t currGoal;

    // Fill in the TE's parameter structure

    current_dir = 1;

    for (;;) {
        usleep(200000); // 200 ms delay

        pthread_testcancel();

        // call
        ProcessMessages();

        // are we waiting for a stall to clear?
        if (waiting) {
            PutPositionCmd(0, 0);
            stall = true;
            PLAYER_MSG0(0, "TE::Main waiting");
            continue;
        }

        // do we have a goal?
        if (!active_goal) {
            continue;
        }

        // wzgledne polozenie celu
        relativeGoal.px = goal.px - odom_pose.px;
        relativeGoal.py = goal.py - odom_pose.py;
        relativeGoal.pa = goal.pa;

        // angle from 0 to the goal (theta)
        theta = atan2(relativeGoal.py, relativeGoal.px);
        // diff betwean robot orientation angle (psi) and goal vector (theta)
        alpha = angle_diff(theta, odom_pose.pa);
        g_dx = hypot(relativeGoal.px, relativeGoal.py);


        if (obstacle && g_dx > dist_eps) {
            //PLAYER_MSG0(1, "TE: obstacle avoidance");
            if (fabs(beta) > ang_eps)
                phi = angle_diff(fabs(beta) / beta * M_PI / 2,
                    angle_diff(beta, alpha));
            else
                phi = angle_diff(M_PI / 2, angle_diff(beta, alpha));
            currGoal.px = cos(phi) * relativeGoal.px +
                    sin(phi) * relativeGoal.py;
            currGoal.py = -sin(phi) * relativeGoal.px +
                    cos(phi) * relativeGoal.py;
            currGoal.pa = relativeGoal.pa;
        } else
            currGoal = relativeGoal;

        // angle from 0 to the goal (theta)
        theta = atan2(currGoal.py, currGoal.px);
        // diff betwean robot orientation angle (psi) and goal vector (theta)
        alpha = angle_diff(theta, odom_pose.pa);
        // are we at the goal?
        g_dx = hypot(currGoal.px, currGoal.py);
        g_da = angle_diff(currGoal.pa, odom_pose.pa);

        if ((g_dx < dist_eps)) { // jestesmy bliko celu
            PLAYER_MSG0(0, "TE::Main Close to goal");
            if (fabs(g_da) < ang_eps) { // z wlasciwa orientacja
                active_goal = false;
                PutPositionCmd(0.0, 0.0);
                PLAYER_MSG0(0, "TE::Main At goal");
                continue;
            } else { // trzeba poprawić orientację po dojechaniu do celu
                PLAYER_MSG0(0, "TE::Main Correcting orientation");
                vx = 0;
                va = k_a * va_max * tanh(10 * g_da);
            }
        } else {
            // sterowanie
            vx = vx_max * tanh(fabs(g_dx)) * fabs(cos(alpha));
            va = k_a * va_max * tanh(alpha);
        }

        if (nearObstDist <= obs_dist) {
            vx = vx * (nearObstDist - min_dist) / (obs_dist - min_dist);
        }
        if (nearObstDist <= min_dist) {
            vx = 0;
            va = 0;
        }

        PutPositionCmd(vx, va);
    }
}

// computes the signed minimum difference between the two angles.

double
TE::angle_diff(double a, double b) {
    double d1, d2;
    a = NORMALIZE(a);
    b = NORMALIZE(b);
    d1 = a - b;
    d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0)
        d2 *= -1.0;
    if (fabs(d1) < fabs(d2))
        return (d1);
    else
        return (d2);
}

////////////////////////////////////////////////////////////////////////////////
// Extra stuff for building a shared object.
#if 1
/* need the extern to avoid C++ name-mangling  */
extern "C" {

    int player_driver_init(DriverTable* table) {
        PLAYER_MSG0(0, "TE driver initializing");
        TE_Register(table);
        PLAYER_MSG0(0, "TE initialization done");
        return (0);
    }
}
#endif
