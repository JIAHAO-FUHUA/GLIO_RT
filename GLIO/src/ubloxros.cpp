#include "utils/common.h"
#include "utils/math_tools.h"
#include "utils/timer.h"
#include "utils/random_generator.hpp"
#include <nav_msgs/Odometry.h>

#include <std_msgs/String.h>
#include <nlosExclusion/GNSS_Raw_Array.h>
#include <sensor_msgs/NavSatFix.h>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <gnss_comm/GnssMeasMsg.h>
#include <gnss_comm/GnssEphemMsg.h>
#include <gnss_comm/GnssGloEphemMsg.h>
#include <gnss_comm/StampedFloat64Array.h>

//#include <ublox_driver/ublox_message_processor.hpp>

// GNSS related header files 
#include <nlosExclusion/GNSS_Raw_Array.h>
#include <nlosExclusion/GNSS_Raw.h>
#include <nlosExclusion/GNSS_Raw_mf.h>
#include "utils/gnss_tools.h"

#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/GnssPVTSolnMsg.h>
#include <rtcm_msgs/Message.h>  


#include <std_msgs/Float32MultiArray.h>


// #include <rtkcmn.h>

// /* RTKLIB Library */
#include "../../GraphGNSSLibV1.1/global_fusion/RTKLIB/src/rtklib.h"

#include <random>
#include <rtcm_msgs/Message.h>

/* tools used for gnss related calculation */
//using namespace gnss_comm;
#define PI 3.1415926535898
#define NFREQ       3

#define REL_HUMI       0.7
#define ERR_SAAS       0.3


const static double gpst0[] = {1980, 1, 6, 0, 0, 0}; /* gps time reference */
const static double gst0[] = {1999, 8, 22, 0, 0, 0}; /* galileo system time reference */
const static double bdt0[] = {2006, 1, 1, 0, 0, 0};  /* beidou time reference */
double lastGNSSTime=0;
double stateecek[3]={-2414266.9197,5386768.9868,2407460.0314};

gnss_comm::GnssEphemMsg eph_msg;


#define P2_43 1.1368683772161603e-13
#define SC2RAD 3.1415926535898
#define P2_55 2.7755575615628914e-17
#define P2_31 4.656612873077393e-10
#define P2_5 3.125
#define P2_29 1.862645149230957e-9
#define P2_19 1.9073486328125e-6
#define P2_33 1.1641532182693481e-10

struct GpsEph {
    int satellite;
    int GPSweek;
    uint8_t URAindex;
    uint8_t flags;
    double IDOT;
    uint8_t IODE;
    double TOC;
    double clock_driftrate;
    double clock_drift;
    double clock_bias;
    uint16_t IODC;
    double Crs;
    double Delta_n;
    double M0;
    double Cuc;
    double e;
    double Cus;
    double sqrt_A;
    double TOE;
    double Cic;
    double OMEGA0;
    double Cis;
    double i0;
    double Crc;
    double omega;
    double OMEGADOT;
    double TGD;
    uint8_t SVhealth;
};



class UbloxRos
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_pvt_;

    ros::Subscriber sub_glo_ephem;
    ros::Subscriber sub_iono;
    ros::Subscriber sub_meas;
    ros::Subscriber sub_time_pulse;
    ros::Subscriber sub_ephem;
    ros::Subscriber sub_rtcm;
    ros::Subscriber sub_rover_;
    ros::Subscriber sub_ephem_;
    //ros::Subscriber sub_rtcm_;

    ros::Subscriber sub_ephem_array;

    ros::Publisher pub_odom_;
    ros::Publisher pub_gnss_raw;
    ros::Publisher pub_ephes;
    ros::Publisher pub_station_raw;

    std::map<int, Eigen::Vector3d> satellite_positions;
    std::map<int, Eigen::Vector3d> satellite_vel;
    std::map<int, Eigen::Vector3d> satellite_dt;
    std::map<int, std::pair<double, double>> azimuth_elevation_map;

    const double mu = 3.986005e14;              // Earth’s universal gravitational parameter
    const double Omega_e_dot = 7.2921151467e-5; // Earth's rotation rate
    const double mug = 3.986004418e14;          // Earth’s universal gravitational parameter for GLONASS
    const double J2 = 1.0826257e-3;             // Second zonal harmonic of the Earth's gravitational potential
    const double Re = 6378136.0;                // Earth's equatorial radius in meters

    std::map<uint32_t, std::vector<gnss_comm::GloEphemPtr>> GloEphemPtrmap;
    std::map<uint32_t, std::vector<gnss_comm::EphemPtr>> EphemPtrmap;
    std::map<uint32_t, std::vector<gnss_comm::SatStatePtr>> satephem;
    gnss_comm::GnssEphemMsgarray rec_ephe;
    nlosExclusion::GNSS_Raw_Array gnss_station_data; // station data to be published
    //GpsEph eph = {0};

    int actual_week = 0;
    bool first_epoch = true;

public:
    UbloxRos() : nh_("~")
    {
        // Initialize subscriber
        sub_pvt_ = nh_.subscribe<gnss_comm::GnssPVTSolnMsg>("/ublox_driver/receiver_pvt", 100, &UbloxRos::ubloxrtklibOdomHandler, this);
        ROS_INFO("UbloxRos constructor finished.");

        // 初始化订阅器和发布器
        //ros::Subscriber sub_pvt_ = nh_.subscribe("/ublox_driver/receiver_pvt", 500, &UbloxRos::ubloxrtklibOdomHandler, this);
        sub_ephem = nh_.subscribe("/ublox_driver/ephem", 1000, &UbloxRos::ephemCallback, this);
        
        
        sub_glo_ephem = nh_.subscribe("/ublox_driver/glo_ephem", 1000, &UbloxRos::gloEphemCallback, this);
        sub_iono = nh_.subscribe("/ublox_driver/iono_params", 1000, &UbloxRos::ionoCallback, this);
        sub_meas = nh_.subscribe("/ublox_driver/range_meas", 1000, &UbloxRos::measCallback, this);
        sub_ephem_array = nh_.subscribe("/ublox_driver/gnssephemarray", 10000, &UbloxRos::gnssephemarrayCallback, this);
        sub_rtcm = nh_.subscribe("/rtcm", 1000, &UbloxRos::rtcmback, this);
        



        //sub_rover_ = nh_.subscribe<gnss_comm::GnssObservations>("/ublox_driver/ipnl/gnss_rover/observations",1000,&UbloxRos::ubloxrover_, this);
        // pub_ephem_ = nh_.advertise<GnssEphemMsg>("ephem", 100);
        // pub_glo_ephem_ = nh_.advertise<GnssGloEphemMsg>("glo_ephem", 100);
        //sub_ephem_ = nh_.advertise<gnss_comm::GnssEphemerides>("/ublox_driver/ipnl/gnss_ephemeris/ephemerides",1000,&UbloxRos::ubloxephem_, this);
        //sub_rtcm = nh_.subscribe("/rtcm", 10000, &UbloxRos::rtcmback, this);
        //sub_time_pulse = nh_.subscribe("ublox_driver/time_pulse_info", 1000, &UbloxRos::timePulseInfoCallback, this);

        //ros::Publisher rtcm_pub = nh_.advertise<rtcm_msgs::Message>("rtcm", 10);

        pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/gnss_preprocessor_node/ECEFSolutionRTK", 100);

        pub_ephes = nh_.advertise<gnss_comm::GnssEphemMsg>("/ephe", 500);
        pub_gnss_raw = nh_.advertise<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/GNSSPsrCarRov1", 1000);
        pub_station_raw = nh_.advertise<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/GNSSPsrCarStation1", 100);

        // pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/gnss_preprocessor_node/ECEFSolutionRTK", 500);

        // pub_ephes = nh_.advertise<gnss_comm::GnssEphemMsg>("/ephe", 500);
        // pub_gnss_raw = nh_.advertise<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/GNSSPsrCarRov1", 1000);
        // pub_station_raw = nh_.advertise<nlosExclusion::GNSS_Raw_Array>("/gnss_preprocessor_node/GNSSPsrCarStation1", 1000);
        

        ROS_INFO("UbloxRos constructor finished.");
    }

    gnss_comm::StampedFloat64Array iono_msg;
    double lla_pvt[3];



    GNSS_Tools gnss_tools_1;   //GNSS工具对象

    double gps2utc(double gps_week, double gps_second)
    {
        return (gps_week * 604800.0 + gps_second - 18.0) + 315964800.0;
    }

    // void rtcmback (const rtcm_msgs::Message::ConstPtr &msg)
    // {
    //     rtcm_t *rtcm;
    //     int ret;
    //     uint8_t data;
    //     data = msg->message;
    //     ret = input_rtcm3(&rtcm, data);
    //     if (ret != 1)
    //         return;
    //     ROS_INFO("satellite number: %d", ret.ephsat);
    // }
    // void rtcmback(const rtcm_msgs::Message::ConstPtr &msg)
    // {
    // }


    void ubloxrtklibOdomHandler(const gnss_comm::GnssPVTSolnMsg::ConstPtr &odomIn)
    {
        Eigen::Matrix<double, 3, 1> LLA;
        LLA << odomIn->longitude, odomIn->latitude, odomIn->altitude;
        lla_pvt[0] = odomIn->latitude;
        lla_pvt[1] = odomIn->longitude;
        lla_pvt[2] = odomIn->altitude;

        Eigen::Matrix<double, 3, 1> ECEF = gnss_tools_1.llh2ecef(LLA); // 将LLH格式的基准点转换为ECEF
        Eigen::Matrix<double, 3, 1> ENU_ref;
        ENU_ref << 0, 0, 0;
        Eigen::Matrix<double, 3, 1> ENU = gnss_tools_1.ecef2enu(ENU_ref, ECEF); // 当前的ECEF坐标转换为ENU坐标

        ::gtime_t cur_t= gpst2time(odomIn->time.week, odomIn->time.tow);
        double cur_time = cur_t.time + cur_t.sec;


        // double cur_time = gpst2time(odomIn->time.week, odomIn->time.tow);

        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.header.stamp = ros::Time(cur_time);
        odometry.child_frame_id = "map";
        odometry.pose.pose.position.x = ECEF(0); // ENU(0);
        odometry.pose.pose.position.y = ECEF(1); // ENU(1);
        odometry.pose.pose.position.z = ECEF(2); // ENU(2);
        odometry.pose.covariance[0] = odomIn->h_acc;
        odometry.pose.covariance[1] = odomIn->h_acc;
        odometry.pose.covariance[2] = odomIn->v_acc;

        // ROS_INFO("llh: %.8f %.8f %.3f", LLA[0], LLA[1], LLA[2]);
        //ROS_INFO("llh: %d %d %d", odomIn->latitude, odomIn->longitude, odomIn->altitude);

        pub_odom_.publish(odometry);
    }

    void ionoCallback(const gnss_comm::StampedFloat64Array::ConstPtr &msg)
    {
        iono_msg.header.stamp = msg->header.stamp; 
        iono_msg.data = msg->data;   
    }

    void gnssephemarrayCallback(const gnss_comm::GnssEphemMsgarray::ConstPtr &msg)
    {
        rec_ephe = *msg;
        //ROS_INFO("msg               %d", rec_ephe.ephem.size());
        // int sat = msg->gloephem.sat;
        // msg->ephem;
        // eph2pos;
        // eph2clk;
        // epoch2time;
        // sat = msg->ephem.sat;

        // sat_azel;

        // SatStatePtr sat_pet;
        // double azel[2] = {0, 0};
        // sat_azel(ref_ecef, all_sat_states[i]->pos, azel);
        // all_sv_azel.emplace_back(azel[0], azel[1]);

        // EphemPtrmap
        // GloEphemPtrmap


        // GloEphemPtr glo_ephem_sat= msg->gloephem[0];

        // EphemPtr ephem_sat=msg->ephem[0];


        // int eph_n=msg->ephem.size();
        // int gloeph_n=msg->gloephem.size();
        // for (int i = 0; i < eph_n; i++)
        // {
        //     EphemPtrmap[msg->ephem[i].sat] = msg->ephem[i];
        // }

        // for (int i = 0; i < gloeph_n; i++)
        // {
        //     GloEphemPtrmap[msg->gloephem[i].sat] = msg->gloephem[i];
        // }

    }


    void gloEphemCallback(const gnss_comm::GnssGloEphemMsg::ConstPtr &msg)
    {
        double tau_n = msg->tau_n;
        double gamma = msg->gamma;
        double pos[3] = {msg->pos_x, msg->pos_y, msg->pos_z};
        double vel[3] = {msg->vel_x, msg->vel_y, msg->vel_z};
        double acc[3] = {msg->acc_x, msg->acc_y, msg->acc_z};
        double ttr= msg->ttr.tow;

        double tk = ttr - msg->toe.tow; // Time from ephemeris reference epoch
        if (tk > 302400)
            tk -= 604800;
        if (tk < -302400)
            tk += 604800;

        // Calculate new position and velocity using numerical integration (Runge-Kutta or similar method)
        // For simplicity, using a basic approach here, which can be replaced with a more accurate method
        double dt = 60.0; // Time step in seconds
        int steps = static_cast<int>(fabs(tk / dt));
        double sign = tk < 0 ? -1.0 : 1.0;

        for (int i = 0; i < steps; ++i)
        {
            double r = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);
            double a = -mug / (r * r * r);
            double ax = a * pos[0] + acc[0];
            double ay = a * pos[1] + acc[1];
            double az = a * pos[2] + acc[2];

            // J2 perturbation
            double z2 = pos[2] * pos[2];
            double r2 = r * r;
            double tx = pos[0] * (5.0 * z2 / r2 - 1.0);
            double ty = pos[1] * (5.0 * z2 / r2 - 1.0);
            double tz = pos[2] * (5.0 * z2 / r2 - 3.0);
            double j2 = 1.5 * J2 * mug * Re * Re / (r * r * r * r);

            ax += j2 * tx;
            ay += j2 * ty;
            az += j2 * tz;

            vel[0] += sign * ax * dt;
            vel[1] += sign * ay * dt;
            vel[2] += sign * az * dt;

            pos[0] += sign * vel[0] * dt;
            pos[1] += sign * vel[1] * dt;
            pos[2] += sign * vel[2] * dt;
        }

        ROS_INFO("gloEphemCallback called for satellite %d", msg->sat);
        // 存储卫星位置
        satellite_positions[msg->sat] = {pos[0], pos[1], pos[2]};
        satellite_vel[msg->sat] = {vel[0], vel[2], vel[2]};
        satellite_dt[msg->sat] = {msg->tau_n, msg->gamma, msg->delta_tau_n};
    }

    // Convert degrees to radians
    double deg2rad(double degrees)
    {
        return degrees * (PI / 180.0);
    }

    // Normalize angle to the range [0, 2*PI)
    double normalize_angle(double angle)
    {
        while (angle < 0)
            angle += 2 * PI;
        while (angle >= 2 * PI)
            angle -= 2 * PI;
        return angle;
    }

    // Calculate satellite position and velocity in ECEF
    void ephemCallback(const gnss_comm::GnssEphemMsg::ConstPtr &msg)
    {
        double pos[3] = {0};
        double vel[3] = {0};
        double acc[3] = {0};
        double A = msg->A;
        double n0 = sqrt(mu / (A * A * A)); // Computed mean motion (rad/s)
        double toe = msg->toe.tow;
        double delta_n = msg->delta_n;
        double M0 = msg->M0;
        double e = msg->e;
        double sqrtA = sqrt(A);
        double ttr= msg->ttr.tow;

        // Time from ephemeris reference epoch
        double tk = ttr - toe;
        if (tk > 302400)
            tk -= 604800;
        if (tk < -302400)
            tk += 604800;

        // Mean anomaly
        double n = n0 + delta_n;
        double M = M0 + n * tk;
        M = normalize_angle(M);

        // Solve Kepler's equation for E (eccentric anomaly)
        double E = M;
        double E_old;
        do
        {
            E_old = E;
            E = M + e * sin(E);
        } while (fabs(E - E_old) > 1e-10);

        // True anomaly
        double sinE = sin(E);
        double cosE = cos(E);
        double v = atan2(sqrt(1 - e * e) * sinE, cosE - e);

        // Argument of latitude
        double u = v + msg->omg;

        // Radius
        double r = A * (1 - e * cosE);

        // Inclination
        double i = msg->i0 + msg->i_dot * tk;

        // Positions in orbital plane
        double x_orb = r * cos(u);
        double y_orb = r * sin(u);

        // Corrected longitude of ascending node
        double Omega = msg->OMG0 + (msg->OMG_dot - Omega_e_dot) * tk - Omega_e_dot * toe;

        // Satellite position in ECEF
        pos[0] = x_orb * cos(Omega) - y_orb * cos(i) * sin(Omega);
        pos[1] = x_orb * sin(Omega) + y_orb * cos(i) * cos(Omega);
        pos[2] = y_orb * sin(i);

        // Time derivatives for velocity calculation
        double edot = n / (1 - e * cosE);
        double vdot = sqrt(1 - e * e) * edot / (1 - e * cosE);

        // Satellite velocity in the orbital plane
        double rdot = A * e * sinE * edot;
        double udot = vdot + msg->omg;

        double xdot_orb = rdot * cos(u) - r * sin(u) * udot;
        double ydot_orb = rdot * sin(u) + r * cos(u) * udot;

        // Satellite velocity in ECEF
        vel[0] = (xdot_orb * cos(Omega) - ydot_orb * cos(i) * sin(Omega)) - Omega_e_dot * pos[1];
        vel[1] = (xdot_orb * sin(Omega) + ydot_orb * cos(i) * cos(Omega)) + Omega_e_dot * pos[0];
        vel[2] = ydot_orb * sin(i);

        satellite_positions[msg->sat] = {pos[0], pos[1], pos[2]};
        satellite_vel[msg->sat] = {vel[0], vel[1], vel[2]};
        satellite_dt[msg->sat] = {msg->af0, msg->af1, msg->af2};
    }

    void computeAzimuth(std::map<int, std::pair<double, double>> &azimuth_elevation_map)
    {
        for (const auto &sat : satellite_positions)
        {
            int sat_id = sat.first;
            ROS_WARN("Satellite ID %d found in computeAzimuth", sat_id);
            Eigen::Vector3d sat_pos_ecef = sat.second;

            // ECEF 转 ENU
            Eigen::Vector3d receiver_pos_ecef = geo2ecef1(lla_pvt[0], lla_pvt[1], lla_pvt[2]);
            Eigen::Matrix3d R = ecef2enu_rotation(lla_pvt[0], lla_pvt[1]);
            Eigen::Vector3d sat_pos_enu = R * (sat_pos_ecef - receiver_pos_ecef);

            // 计算方位角和高度角
            double azimuth = atan2(sat_pos_enu.y(), sat_pos_enu.x());
            double elevation = atan2(sat_pos_enu.z(), sqrt(sat_pos_enu.x() * sat_pos_enu.x() + sat_pos_enu.y() * sat_pos_enu.y()));

            // 转换为角度
            azimuth *= 180 / PI;
            elevation *= 180 / PI;

            // 存储计算结果
            azimuth_elevation_map[sat_id] = std::make_pair(azimuth, elevation);

            // 输出结果（可选）
            std::cout << "Satellite " << sat_id << ": Azimuth = " << azimuth << "°, Elevation = " << elevation << "°" << std::endl;
        }
    }

    // 计算 ECEF 到 ENU 旋转矩阵
    Eigen::Matrix3d ecef2enu_rotation(double lat, double lon)
    {
        double sin_lat = sin(lat * PI / 180);
        double cos_lat = cos(lat * PI / 180);
        double sin_lon = sin(lon * PI / 180);
        double cos_lon = cos(lon * PI / 180);

        Eigen::Matrix3d R;
        R << -sin_lon, cos_lon, 0,
            -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
        return R;
    }

    // // 计算地理坐标 (lat, lon, alt) 到 ECEF 坐标
    Eigen::Vector3d geo2ecef1(double lat, double lon, double alt)
    {
        double a = 6378137.0;           // 地球长半轴 (WGS-84)
        double e_sq = 6.69437999014e-3; // 偏心率的平方 (WGS-84)

        double sin_lat = sin(lat * PI / 180);
        double cos_lat = cos(lat * PI / 180);
        double sin_lon = sin(lon * PI / 180);
        double cos_lon = cos(lon * PI / 180);

        double N = a / sqrt(1 - e_sq * sin_lat * sin_lat);

        double x = (N + alt) * cos_lat * cos_lon;
        double y = (N + alt) * cos_lat * sin_lon;
        double z = (N * (1 - e_sq) + alt) * sin_lat;

        return Eigen::Vector3d(x, y, z);
    }

    /* transform ecef to geodetic postion ------------------------------------------
     * transform ecef position to geodetic position
     * args   : double *r        I   ecef position {x,y,z} (m)
     *          double *pos      O   geodetic position {lat,lon,h} (rad,m)
     * return : none
     * notes  : WGS84, ellipsoidal height
     *-----------------------------------------------------------------------------*/

    // void ecef2pos(const double *r, double *pos)
    // {
    //     double e2 = FE_WGS84 * (2.0 - FE_WGS84), r2 = dot(r, r, 2), z, zk, v = RE_WGS84, sinp;

    //     for (z = r[2], zk = 0.0; fabs(z - zk) >= 1E-4;)
    //     {
    //         zk = z;
    //         sinp = z / sqrt(r2 + z * z);
    //         v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
    //         z = r[2] + v * e2 * sinp;
    //     }
    //     pos[0] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (r[2] > 0.0 ? PI / 2.0 : -PI / 2.0);
    //     pos[1] = r2 > 1E-12 ? atan2(r[1], r[0]) : 0.0;
    //     pos[2] = sqrt(r2 + z * z) - v;
    // }


    void measCallback(const gnss_comm::GnssMeasMsg::ConstPtr &msg)
    {

        int n=msg->meas.size();

        double *rs, *dts, *var, *azel_, *resp;
        int i, stat, vsat[MAXOBS] = {0}, svh[MAXOBS];


        rs = mat(6, n);
        dts = mat(2, n);
        var = mat(1, n);
        azel_ = zeros(2, n);
        resp = mat(1, n);
        nlosExclusion::GNSS_Raw_Array gnss_data; // revise

        if (n <= 0)
        {
            std::cerr << "Error: no measurements available. Exiting callback." << std::endl;
            return ;
        }

        int current_week = msg->meas[0].time.week;
        double current_tow = msg->meas[0].time.tow;
        double epoch_time[100];
        gnss_comm::gtime_t current_time = gnss_comm::gpst2time(msg->meas[0].time.week, msg->meas[0].time.tow);
        gnss_comm::time2epoch(current_time, epoch_time);

        if (first_epoch) {
            actual_week = current_week;
            first_epoch = false;
        }
        else {
            if (actual_week != current_week)
            {
                std::cout << "gnss week not aligned!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
                exit(0);
            }
        }


        /* estimate receiver position with pseudorange by WLS and Eigen */
        bool haveOneBeiDou = false;
        int CMP_cnt = 0, GPS_cnt = 0, GAL_cnt = 0, GLO_cnt = 0;

        //std::map<int, std::pair<double, double>> azimuth_elevation_map;
        //computeAzimuth(azimuth_elevation_map);

        for (int s_i = 0; s_i < n; s_i++)
        {
            nlosExclusion::GNSS_Raw gnss_raw;
            gnss_raw.GNSS_time = current_tow;
            gnss_raw.prE3dMA = current_week;
            gnss_raw.GNSS_week = current_week;
            gnss_raw.total_sv = float(n);
            gnss_raw.prn_satellites_index = float(msg->meas[s_i].sat);

            nlosExclusion::GNSS_Raw_mf gnss_raw_mf;
            gnss_raw_mf.GNSS_week = current_week;
            gnss_raw_mf.GNSS_time = current_tow;
            gnss_raw_mf.total_sv = float(n);
            gnss_raw_mf.prn_satellites_index = float(msg->meas[s_i].sat);

            //NFREQ=3
            /* get snr*/
            double snr = 0;
            for (int j = 0, snr = 0.0; j < NFREQ; j++)
                if ((snr = msg->meas[s_i].CN0[j]) != 0.0)
                    break;
            gnss_raw.snr = msg->meas[s_i].CN0[0] * 0.25;
            // gnss_raw.snr = snr;

            // 更新方位角和高度角
            int sat_id = msg->meas[s_i].sat;
            //ROS_INFO("sat star.");


            // EphemPtrmap[msg->ephem[i].sat] = msg->ephem[i];

            // GloEphemPtrmap[msg->gloephem[i].sat] = msg->gloephem[i];

            Eigen::Vector3d sat_ecef;
            Eigen::Vector3d sat_vel;
            double svdt;
            double svddt;
            double azel[2];
            Eigen::Vector3d receiver_pos_ecef = geo2ecef1(lla_pvt[0], lla_pvt[1], lla_pvt[2]);
            const uint32_t sys = satsys(sat_id, NULL);

            if(sys == SYS_GLO)
            {
                gnss_comm::GloEphemPtr ephemPtr_;
                for (auto iter = rec_ephe.gloephem.begin();
                     iter != rec_ephe.gloephem.end();)
                {
                    if (iter->sat == sat_id)
                    {
                        //gnss_comm::GnssGloEphemMsg ephemPtr_msg= *iter;
                        auto temp_ptr = boost::make_shared<gnss_comm::GnssGloEphemMsg>(*iter);

                        ephemPtr_ = gnss_comm::msg2glo_ephem(temp_ptr);

                        svdt = iter->tau_n;
                        svddt = iter->gamma;

                        sat_ecef = gnss_comm::geph2pos(current_time, ephemPtr_, &svdt);
                        sat_vel = gnss_comm::geph2vel(current_time, ephemPtr_, &svddt);


                        gnss_raw.sat_clk_err = iter->tau_n * CLIGHT; // CLIGHT=299792458.0  revise
                        gnss_raw.ttx = 0;
                        gnss_raw.dt = iter->gamma * CLIGHT;
                        gnss_raw.ddt = iter->delta_tau_n * CLIGHT;
                        gnss_raw.tgd = 0;

                        gnss_raw.sat_pos_x = sat_ecef[0];
                        gnss_raw.sat_pos_y = sat_ecef[1];
                        gnss_raw.sat_pos_z = sat_ecef[2];
                        gnss_raw.vel_x = sat_vel[0];
                        gnss_raw.vel_y = sat_vel[1];
                        gnss_raw.vel_z = sat_vel[2];
                        gnss_comm::sat_azel(receiver_pos_ecef, sat_ecef, azel);
                        //ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);

                        gnss_raw.azimuth = azel[0] * 180 / PI;
                        gnss_raw.elevation = azel[1] * 180 / PI;
                        gnss_raw_mf.azimuth = gnss_raw.azimuth;
                        gnss_raw_mf.elevation = gnss_raw.elevation;
                        azel_[0 + s_i * 2] = azel[0];
                        azel_[1 + s_i * 2] = azel[1];

                        break;
                    }
                    else
                    {
                        ++iter;
                    }
                }

            }
            else 
            {
                gnss_comm::EphemPtr ephemPtr_; //gloephem
                for (auto iter = rec_ephe.ephem.begin();
                     iter != rec_ephe.ephem.end();)
                {
                    if (iter->sat == sat_id)
                    {
                        //gnss_comm::GnssEphemMsg ephemPtr_msg= *iter;
                        auto temp_ptr = boost::make_shared<gnss_comm::GnssEphemMsg>(*iter);
                        ephemPtr_ = gnss_comm::msg2ephem(temp_ptr);

                        //ephemPtr_ = gnss_comm::msg2ephem(ephemPtr_msg);

                        svdt = iter->af0;
                        svddt = iter->af1;

                        sat_ecef = gnss_comm::eph2pos(current_time, ephemPtr_, &svdt);
                        sat_vel = gnss_comm::eph2vel(current_time, ephemPtr_, &svddt);

                        gnss_raw.sat_clk_err = iter->af0 * CLIGHT; // CLIGHT=299792458.0  revise
                        gnss_raw.ttx = 0;
                        gnss_raw.dt = iter->af1 * CLIGHT;
                        gnss_raw.ddt = iter->af2 * CLIGHT;
                        gnss_raw.tgd = 0;

                        gnss_raw.sat_pos_x = sat_ecef[0];
                        gnss_raw.sat_pos_y = sat_ecef[1];
                        gnss_raw.sat_pos_z = sat_ecef[2];
                        gnss_raw.vel_x = sat_vel[0];
                        gnss_raw.vel_y = sat_vel[1];
                        gnss_raw.vel_z = sat_vel[2];
                        gnss_comm::sat_azel(receiver_pos_ecef, sat_ecef, azel);

                        gnss_raw.azimuth = azel[0] * 180 / PI;
                        gnss_raw.elevation = azel[1] * 180 / PI;
                        gnss_raw_mf.azimuth = gnss_raw.azimuth;
                        gnss_raw_mf.elevation = gnss_raw.elevation;
                        azel_[0 + s_i * 2] = azel[0];
                        azel_[1 + s_i * 2] = azel[1];

                        //ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);
                        break;
                    }
                    else
                    {
                        ++iter;
                    }
                }
            }



            double dion, dtrp, vmeas, vion, vtrp, rr[3], pos[3], e[3], P, lam_L1;
            /* psudorange with code bias correction */
            // if ((P = prange(obs + s_i, nav, azel_ + s_i * 2, 2, &opt_, &vmeas)) == 0.0)  //revise
            //     continue;

            /* ionospheric corrections */
            for (int i = 0; i < 3; i++)
                rr[i] = lla_pvt[i];
            ecef2pos(rr, pos);
            // if (!ionocorr(obs[s_i].time, nav, obs[s_i].sat, pos, azel_ + s_i * 2,
            //               opt->ionoopt, &dion, &vion))
            //     continue;
            
            // *ion=ionmodel(time,nav->ion_gps,pos,azel); //revise
            // *var=SQR(*ion*ERR_BRDCI);
            
            ::gtime_t time_cur = gpst2time(current_week, current_tow);
            dion=ionmodel(time_cur,iono_msg.data.data(),pos,azel_ + s_i * 2); //revise careful
            vion=(dion*0.5)*(dion*0.5);

            // ROS_INFO("ionmodel finish.");
            // printf("dion-> WLS %f \n", dion);
            // printf("dtrp %f \n", dtrp);

            /* GPS-L1 -> L1/B1 */
            double lam_nav=299792458.0/msg->meas[s_i].freqs[0];
            if ((lam_nav) > 0.0)
            {
                dion *= (lam_nav / msg->meas[s_i].cp[0])*(lam_nav / msg->meas[s_i].cp[0]);
            }

            /* tropospheric corrections
             * pose of receievr.
             */
            // if (!tropcorr(obs[s_i].time, nav, pos, azel_ + s_i * 2,
            //               opt->tropopt, &dtrp, &vtrp))
            // {
            //     continue;
            // }

            dtrp=tropmodel(time_cur,pos,azel_,REL_HUMI); //REL_HUMI=0.7
            vtrp=(ERR_SAAS/(sin(azel_[1])+0.1))*(ERR_SAAS/(sin(azel_[1])+0.1));  //ERR_SAAS=0.3

            // ROS_INFO("tropmodel finish.");
            // printf("dtrp %f \n", dtrp);
            gnss_raw.err_tropo = dtrp;
            gnss_raw.err_iono = dion;


            gnss_raw_mf.err_tropo = dtrp;
            gnss_raw_mf.sat_clk_err = gnss_raw.sat_clk_err;  
            gnss_raw_mf.ttx = 0;
            gnss_raw_mf.dt = gnss_raw.dt;
            gnss_raw_mf.ddt = gnss_raw.ddt;
            gnss_raw_mf.tgd = 0;
            gnss_raw_mf.sat_pos_x = gnss_raw.sat_pos_x;
            gnss_raw_mf.sat_pos_y = gnss_raw.sat_pos_y;
            gnss_raw_mf.sat_pos_z = gnss_raw.sat_pos_z;
            gnss_raw_mf.vel_x = gnss_raw.vel_x;
            gnss_raw_mf.vel_y = gnss_raw.vel_y;
            gnss_raw_mf.vel_z = gnss_raw.vel_z;

            /* get pr*/
            double pr = 0;
            for (int j = 0; j < NFREQ; j++)
                if ((pr = msg->meas[s_i].psr[j]) != 0.0)
                    break;
            gnss_raw.pseudorange = msg->meas[s_i].psr[0]; //revise
            // gnss_raw.pseudorange = pr;
            // gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;
            /* remove the satellite clock bias, atmosphere error here */
            //gnss_raw.pseudorange = P + gnss_raw.sat_clk_err - dion - dtrp;  //revise

            gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;  

            gnss_raw.raw_pseudorange = msg->meas[s_i].psr[0];
            gnss_raw.carrier_phase = msg->meas[s_i].cp[0];
            gnss_raw.doppler = msg->meas[s_i].dopp[0];
            gnss_raw.LLI = msg->meas[s_i].LLI[0];
            gnss_raw.slip = (msg->meas[s_i].LLI[0] & 3) > 0 ? 1 : 0;
            gnss_raw.lamda = lam_nav;

            //int sys = satsys(msg->meas[s_i].sat, NULL);
            gnss_raw_mf.constellation = sys;

            for (int j = 0; j < NFREQ + NEXOBS; j++)
            {
                
                // gnss_raw_mf.lamda.push_back(nav->lam[obs[s_i].sat - 1][j]);
                // gnss_raw_mf.err_iono.push_back(dion * SQR(nav->lam[obs[s_i].sat - 1][j] / nav->lam[obs[s_i].sat - 1][0]));
                // // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
                // gnss_raw_mf.pseudorange.push_back(1.0);
                // //            gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
                // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
                // gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
                // gnss_raw_mf.doppler.push_back(obs[s_i].D[j]);
                // gnss_raw_mf.snr.push_back(obs[s_i].SNR[j] * 0.25);


                double lam_LL = 299792458.0/msg->meas[s_i].freqs[j];
                gnss_raw_mf.lamda.push_back(lam_LL);
                gnss_raw_mf.err_iono.push_back(dion * (lam_LL / lam_nav)*(lam_LL / lam_nav));
                gnss_raw_mf.pseudorange.push_back(1.0);
                gnss_raw_mf.raw_pseudorange.push_back(msg->meas[s_i].psr[j]);
                gnss_raw_mf.raw_carrier_phase.push_back(msg->meas[s_i].cp[j]);
                gnss_raw_mf.doppler.push_back(msg->meas[s_i].dopp[j]);
                gnss_raw_mf.snr.push_back(msg->meas[s_i].CN0[j] * 0.25);


            }

            if (gnss_raw.elevation > 15) // 15
            {
                gnss_data.GNSS_Raws.push_back(gnss_raw);
                gnss_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
            }

#if 1 // debug satellite information
            if (sys == SYS_GPS)
            {
                GPS_cnt++;
            }
            else if (sys == SYS_CMP)
            {
                haveOneBeiDou = true;
                CMP_cnt++;
            }
            else if (sys == SYS_GAL)
            {
                GAL_cnt++;
            }
            else if (sys == SYS_GLO)
            {
                GLO_cnt++;
            }
#endif
        }

        // Ensure publisher is valid before publishing
        if (pub_gnss_raw)
        {
            if (fabs(current_tow - lastGNSSTime) > 0.0)
            {
                if (gnss_data.GNSS_Raws.size() != 0)
                {
                    pub_gnss_raw.publish(gnss_data);
                    lastGNSSTime = current_tow;
                }
                // pub_gnss_raw.publish(gnss_data);
                // lastGNSSTime = current_tow;
            }
            //ROS_ERROR("publish gnss_data.");
        }
        else
        {
            ROS_ERROR("Publisher is invalid, cannot publish gnss_data.");
        }

        // if (fabs(current_tow - lastGNSSTime) > 0.0)
        // {
        //     pub_gnss_raw.publish(gnss_data);
        //     lastGNSSTime = current_tow;
        // }
        // ROS_ERROR("publish gnss_data.");
    }

    void rtcmback(const rtcm_msgs::Message::ConstPtr &msg)
    {
        rtcm_t rtcm; // 创建 rtcm_t 结构
        int ret;

        // 初始化 rtcm_t 结构
        memset(&rtcm, 0, sizeof(rtcm_t));
        if (!init_rtcm(&rtcm))
        {
            ROS_ERROR("Failed to initialize RTCM structure");
            return;
        }

        for (const auto &data : msg->message)
        {
            ret = input_rtcm3(&rtcm, data);
            //ROS_INFO("satellite number: %d", rtcm.obs.n);
        }
        // ROS_INFO("ephe update: %d", rtcm.ephsat);
        // ROS_INFO("nav number: %d", rtcm.nav.n);
        // ROS_INFO("glonav number: %d", rtcm.nav.ng);
        // ROS_INFO("sbas number: %d", rtcm.nav.ns);

        // ROS_INFO("ephe sat                   : %d", rtcm.nav.peph[0].pos[rtcm.obs.data[0].sat][0]);
        //ROS_INFO("gloephe sat                   : %d", rtcm.nav.geph[0].week);


        // ROS_INFO("all number: %d", rtcm.obs.n);
        // ROS_INFO("satellite sat: %d", rtcm.obs.data[rtcm.obs.n-1].sat);
        //  ROS_INFO("all time: %d", rtcm.time.time);
        // ROS_INFO("all sat time: %d", rtcm.obs.data[rtcm.obs.n-1].time.time);
        //  ROS_INFO("rtk_pos             : %.f %.f %.f %.f", rtcm.sta.pos[0],rtcm.sta.pos[1],rtcm.sta.pos[2],rtcm.sta.hgt);
        // ROS_INFO("rtk_pos_del         : %d %d %d %d %d", rtcm.sta.antsetup, rtcm.sta.deltype,rtcm.sta.del[0],rtcm.sta.del[1],rtcm.sta.del[2]);


        /* temporal update of states
         * cycle slip detection can also be done here.
         */
        //udstate(rtk, obs, sat, iu, ir, ns, nav);
        int ns=rtcm.obs.n;
        std::cout << "ns: " << ns << endl;

        if (rtcm.obs.n == 0 ) return ;
        // nlosExclusion::GNSS_Raw_Array gnss_data; // station data to be published
        int current_week = 0;
        double current_tow;
        current_tow = time2gpst(rtcm.obs.data[0].time, &current_week);

        if (actual_week != current_week) {
            current_week = actual_week;
        }

        gnss_comm::gtime_t current_time = gnss_comm::gpst2time(current_week, current_tow);

        for (int is = 0; is < ns; is++)
        {
            {
                nlosExclusion::GNSS_Raw gnss_raw;
                gnss_raw.GNSS_time = current_tow;
                gnss_raw.GNSS_week = current_week;
                gnss_raw.total_sv = float(ns); // same satellite with user end
                gnss_raw.prn_satellites_index = float(rtcm.obs.data[is].sat);
                gnss_raw.snr = rtcm.obs.data[is].SNR[0] * 0.25;
                //            LOG(INFO)<<"1785";

                //            gnss_raw.visable = rtk->ssat[obs[ir[is]].sat-1].slip[f]; // sat=obs[i].sat;
                //            LOG(INFO)<<"1788";

                gnss_raw.raw_pseudorange = rtcm.obs.data[is].P[0];
                gnss_raw.carrier_phase = rtcm.obs.data[is].L[0];
                gnss_raw.lamda = rtcm.nav.lam[rtcm.obs.data[is].sat - 1][0];
                gnss_raw.LLI = rtcm.obs.data[is].LLI[0];
                // simple detection based on LLI, we can add other logic to detect CS
                gnss_raw.slip = (rtcm.obs.data[is].LLI[0] & 3) > 0 ? 1 : 0;
                //            LOG(INFO)<<"1795";

                double rr_[3] = {stateecek[0], stateecek[1], stateecek[2]};




                // 更新方位角和高度角
                int sat_id = rtcm.obs.data[is].sat;

                Eigen::Vector3d sat_ecef;
                Eigen::Vector3d sat_vel;
                double svdt;
                double svddt;
                //double azel[3] = {0};
                Eigen::Vector3d receiver_pos_ecef = {rr_[0], rr_[1], rr_[2]};
                const uint32_t sys = satsys(sat_id, NULL);

                if (sys == SYS_GLO)
                {
                    gnss_comm::GloEphemPtr ephemPtr_;
                    for (auto iter = rec_ephe.gloephem.begin();
                         iter != rec_ephe.gloephem.end();)
                    {
                        if (iter->sat == sat_id)
                        {
                            // gnss_comm::GnssGloEphemMsg ephemPtr_msg= *iter;
                            auto temp_ptr = boost::make_shared<gnss_comm::GnssGloEphemMsg>(*iter);

                            ephemPtr_ = gnss_comm::msg2glo_ephem(temp_ptr);

                            svdt = iter->tau_n;
                            svddt = iter->gamma;

                            sat_ecef = gnss_comm::geph2pos(current_time, ephemPtr_, &svdt);
                            //sat_vel = gnss_comm::geph2vel(current_time, ephemPtr_, &svddt);

                            gnss_raw.sat_clk_err = iter->tau_n * CLIGHT; // CLIGHT=299792458.0  revise
                            gnss_raw.ttx = 0;
                            gnss_raw.dt = iter->gamma * CLIGHT;
                            gnss_raw.ddt = iter->delta_tau_n * CLIGHT;
                            gnss_raw.tgd = 0;

                            gnss_raw.sat_pos_x = sat_ecef[0];
                            gnss_raw.sat_pos_y = sat_ecef[1];
                            gnss_raw.sat_pos_z = sat_ecef[2];
                            // gnss_raw.vel_x = sat_vel[0];
                            // gnss_raw.vel_y = sat_vel[1];
                            // gnss_raw.vel_z = sat_vel[2];
                            // gnss_comm::sat_azel(receiver_pos_ecef, sat_ecef, azel);

                            // gnss_raw.azimuth = azel[0] * 180 / PI;
                            // gnss_raw.elevation = azel[1] * 180 / PI;
                            // gnss_raw_mf.azimuth = gnss_raw.azimuth;
                            // gnss_raw_mf.elevation = gnss_raw.elevation;
                            // azel_[0 + s_i * 2] = azel[0];
                            // azel_[1 + s_i * 2] = azel[1];

                            break;
                        }
                        else
                        {
                            ++iter;
                        }
                    }
                }
                else
                {
                    gnss_comm::EphemPtr ephemPtr_; // gloephem
                    for (auto iter = rec_ephe.ephem.begin();
                         iter != rec_ephe.ephem.end();)
                    {
                        if (iter->sat == sat_id)
                        {
                            // gnss_comm::GnssEphemMsg ephemPtr_msg= *iter;
                            auto temp_ptr = boost::make_shared<gnss_comm::GnssEphemMsg>(*iter);
                            ephemPtr_ = gnss_comm::msg2ephem(temp_ptr);

                            // ephemPtr_ = gnss_comm::msg2ephem(ephemPtr_msg);

                            svdt = iter->af0;
                            svddt = iter->af1;

                            sat_ecef = gnss_comm::eph2pos(current_time, ephemPtr_, &svdt);
                            //sat_vel = gnss_comm::eph2vel(current_time, ephemPtr_, &svddt);

                            gnss_raw.sat_clk_err = iter->af0 * CLIGHT; // CLIGHT=299792458.0  revise
                            gnss_raw.ttx = 0;
                            gnss_raw.dt = iter->af1 * CLIGHT;
                            gnss_raw.ddt = iter->af2 * CLIGHT;
                            gnss_raw.tgd = 0;

                            gnss_raw.sat_pos_x = sat_ecef[0];
                            gnss_raw.sat_pos_y = sat_ecef[1];
                            gnss_raw.sat_pos_z = sat_ecef[2];
                            // gnss_raw.vel_x = sat_vel[0];
                            // gnss_raw.vel_y = sat_vel[1];
                            // gnss_raw.vel_z = sat_vel[2];
                            // gnss_comm::sat_azel(receiver_pos_ecef, sat_ecef, azel);

                            // gnss_raw.azimuth = azel[0] * 180 / PI;
                            // gnss_raw.elevation = azel[1] * 180 / PI;
                            // gnss_raw_mf.azimuth = gnss_raw.azimuth;
                            // gnss_raw_mf.elevation = gnss_raw.elevation;
                            // azel_[0 + s_i * 2] = azel[0];
                            // azel_[1 + s_i * 2] = azel[1];

                            // ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);
                            break;
                        }
                        else
                        {
                            ++iter;
                        }
                    }
                }



                // gnss_raw.sat_clk_err = dts[0 + is * 2] * CLIGHT;
                // gnss_raw.sat_pos_x = rs[0 + is * 6];
                // gnss_raw.sat_pos_y = rs[1 + is * 6];
                // gnss_raw.sat_pos_z = rs[2 + is * 6];
                //            LOG(INFO)<<"gnss_raw.sat_pos-> "<<gnss_raw.sat_pos_x << " " << gnss_raw.sat_pos_y << " " << gnss_raw.sat_pos_z;

                //            if (gnss_raw.carrier_phase < 1000) continue;
                double rs_[3] = {gnss_raw.sat_pos_x, gnss_raw.sat_pos_y, gnss_raw.sat_pos_z};
                

                //double rr_[3] = {rtk->rb[0], rtk->rb[1], rtk->rb[2]};
                double e_[3] = {0};
                double r_ = geodist(rs_, rr_, e_);
                double pos_[3] = {0};
                double azel_[3] = {0};
                ecef2pos(rr_, pos_);
                satazel(pos_, e_, azel_);

                gnss_raw.azimuth = azel_[0] * R2D;
                gnss_raw.elevation = azel_[1] * R2D;

                gnss_raw.pseudorange = rtcm.obs.data[is].P[0];
                gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err;
                // if(gnss_raw.raw_pseudorange> 1000)
               // gnss_data.GNSS_Raws.push_back(gnss_raw);

                nlosExclusion::GNSS_Raw_mf gnss_raw_mf;
                gnss_raw_mf.GNSS_week = current_week;
                gnss_raw_mf.GNSS_time = current_tow;
                gnss_raw_mf.total_sv = float(ns); // same satellite with user end
                gnss_raw_mf.prn_satellites_index = float(rtcm.obs.data[is].sat);
                gnss_raw_mf.azimuth = azel_[0] * R2D;
                gnss_raw_mf.elevation = azel_[1] * R2D;

                gnss_raw_mf.sat_clk_err = gnss_raw.sat_clk_err;
                gnss_raw_mf.sat_pos_x = gnss_raw.sat_pos_x;
                gnss_raw_mf.sat_pos_y = gnss_raw.sat_pos_y;
                gnss_raw_mf.sat_pos_z = gnss_raw.sat_pos_z;

                for (int j = 0; j < NFREQ + NEXOBS; j++)
                {
                    gnss_raw_mf.lamda.push_back(rtcm.nav.lam[rtcm.obs.data[is].sat][j]);
                    gnss_raw_mf.pseudorange.push_back(1.0);
                    gnss_raw_mf.raw_pseudorange.push_back(rtcm.obs.data[is].P[j]);
                    gnss_raw_mf.raw_carrier_phase.push_back(rtcm.obs.data[is].L[j]);
                    gnss_raw_mf.doppler.push_back(rtcm.obs.data[is].D[j]);
                    gnss_raw_mf.snr.push_back(rtcm.obs.data[is].SNR[j] * 0.25);
                }

                // gnss_station_data.GNSS_Raws.push_back(gnss_raw);
                // gnss_station_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
                if (gnss_raw.elevation > 15) // 15
                {
                    gnss_station_data.GNSS_Raws.push_back(gnss_raw);
                    gnss_station_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
                }
                //gnss_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
                //int sys = satsys(rtcm.obs.data[is].sat, NULL);
            }
        }        

        if (ret == 1)
        {

            if (gnss_station_data.GNSS_Raws.size() != 0)
            {
                pub_station_raw.publish(gnss_station_data);
            }
            //pub_station_raw.publish(gnss_station_data);

            //ROS_INFO("all number: %d", gnss_station_data.GNSS_Raws.size());
            gnss_station_data.GNSS_Raws.clear();
            gnss_station_data.GNSS_Raws.shrink_to_fit();

            gnss_station_data.GNSS_Raws_mf.clear();
            gnss_station_data.GNSS_Raws_mf.shrink_to_fit();
        }

        // 释放 rtcm_t 结构的内存
        free_rtcm(&rtcm);
    }








    // void measCallback(const gnss_comm::GnssMeasMsg::ConstPtr &msg)
    // {

    //     int n=msg->meas.size();

    //     double *rs, *dts, *var, *azel_, *resp;
    //     int i, stat, vsat[MAXOBS] = {0}, svh[MAXOBS];


    //     rs = mat(6, n);
    //     dts = mat(2, n);
    //     var = mat(1, n);
    //     azel_ = zeros(2, n);
    //     resp = mat(1, n);
    //     nlosExclusion::GNSS_Raw_Array gnss_data; // revise

    //     if (n <= 0)
    //     {
    //         std::cerr << "Error: no measurements available. Exiting callback." << std::endl;
    //         return ;
    //     }

    //     int current_week = msg->meas[0].time.week;
    //     double current_tow = msg->meas[0].time.tow;
    //     double epoch_time[100];
    //     ::gtime_t current_time = gpst2time(msg->meas[0].time.week, msg->meas[0].time.tow);
    //     time2epoch(current_time, epoch_time);


    //     /* estimate receiver position with pseudorange by WLS and Eigen */
    //     bool haveOneBeiDou = false;
    //     int CMP_cnt = 0, GPS_cnt = 0, GAL_cnt = 0, GLO_cnt = 0;

    //     //std::map<int, std::pair<double, double>> azimuth_elevation_map;
    //     //computeAzimuth(azimuth_elevation_map);

    //     for (int s_i = 0; s_i < n; s_i++)
    //     {
    //         nlosExclusion::GNSS_Raw gnss_raw;
    //         gnss_raw.GNSS_time = current_tow;
    //         gnss_raw.prE3dMA = current_week;
    //         gnss_raw.GNSS_week = current_week;
    //         gnss_raw.total_sv = float(n);
    //         gnss_raw.prn_satellites_index = float(msg->meas[s_i].sat);

    //         nlosExclusion::GNSS_Raw_mf gnss_raw_mf;
    //         gnss_raw_mf.GNSS_week = current_week;
    //         gnss_raw_mf.GNSS_time = current_tow;
    //         gnss_raw_mf.total_sv = float(n);
    //         gnss_raw_mf.prn_satellites_index = float(msg->meas[s_i].sat);

    //         //NFREQ=3
    //         /* get snr*/
    //         double snr = 0;
    //         for (int j = 0, snr = 0.0; j < NFREQ; j++)
    //             if ((snr = msg->meas[s_i].CN0[j]) != 0.0)
    //                 break;
    //         gnss_raw.snr = msg->meas[s_i].CN0[0] * 0.25;
    //         // gnss_raw.snr = snr;

    //         // 更新方位角和高度角
    //         int sat_id = msg->meas[s_i].sat;
    //         ROS_INFO("sat star.");


    //         EphemPtrmap[msg->ephem[i].sat] = msg->ephem[i];

    //         GloEphemPtrmap[msg->gloephem[i].sat] = msg->gloephem[i];

    //         Eigen::Vector3d sat_ecef;
    //         Eigen::Vector3d sat_vel;
    //         double svdt;
    //         double svddt;
    //         double azel[2];
    //         Eigen::Vector3d receiver_pos_ecef = geo2ecef1(lla_pvt[0], lla_pvt[1], lla_pvt[2]);

    //         if (EphemPtrmap.find(sat_id) != EphemPtrmap.end())
    //         {
    //             // std::map<uint32_t, std::vector<GloEphemPtr>> GloEphemPtrmap;
    //             // std::map<uint32_t, std::vector<EphemPtr>> EphemPtrmap;
    //             // std::map<uint32_t, std::vector<SatStatePtr>> satephem;
    //             EphemPtr ephemPtr_= EphemPtrmap[sat_id];
    //             svdt= ephemPtr_.af0;
    //             svddt=ephemPtr_.af1;

    //             sat_ecef = geph2pos(current_time, ephemPtr_, svdt);
    //             sat_vel = geph2vel(current_time, ephemPtr_, svddt);

    //             gnss_raw.sat_clk_err = ephemPtr_.af0; // CLIGHT=299792458.0  revise
    //             gnss_raw.ttx = 0;
    //             gnss_raw.dt = ephemPtr_.af1;  
    //             gnss_raw.ddt = ephemPtr_.af2; 
    //             gnss_raw.tgd = 0;

    //             gnss_raw.sat_pos_x = sat_ecef[0];
    //             gnss_raw.sat_pos_y = sat_ecef[1];
    //             gnss_raw.sat_pos_z = sat_ecef[2];
    //             gnss_raw.vel_x = sat_vel[0];
    //             gnss_raw.vel_y = sat_vel[1];
    //             gnss_raw.vel_z = sat_vel[2];
    //             sat_azel(receiver_pos_ecef, sat_ecef, azel);
    //             ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);

    //             gnss_raw.azimuth = azel[0]*180/PI;
    //             gnss_raw.elevation = azel[1]*180/PI;
//                 gnss_raw_mf.azimuth = gnss_raw.azimuth;
//                 gnss_raw_mf.elevation = gnss_raw.elevation;
//                 azel_[0 + s_i*2]=azel[0];
//                 azel_[1 + s_i*2]=azel[1];

//                 // if (sys == SYS_GLO)
//                 //     sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<GloEphem>(best_ephem), NULL);
//                 // else
//                 //     sat_ecef = eph2pos(obs->time, std::dynamic_pointer_cast<Ephem>(best_ephem), NULL);
//                 // double azel[2] = {0, M_PI / 2.0};
//                 // sat_azel(ecef_pos, sat_ecef, azel);
//                 // if (azel[1] < GNSS_ELEVATION_THRES * M_PI / 180.0)
//                 //     continue;

//             }
//             else if (GloEphemPtrmap.find(sat_id) != GloEphemPtrmap.end())
//             {
//                 GloEphemPtr ephemPtr_= GloEphemPtrmap[sat_id];
//                 svdt= ephemPtr_.tau_n;
//                 svddt=ephemPtr_.gamma;

//                 sat_ecef = eph2pos(current_time, ephemPtr_, svdt);
//                 sat_vel = eph2vel(current_time, ephemPtr_, svddt);

//                 gnss_raw.sat_clk_err = ephemPtr_.tau_n; // CLIGHT=299792458.0  revise
//                 gnss_raw.ttx = 0;
//                 gnss_raw.dt = ephemPtr_.gamma;  
//                 gnss_raw.ddt = ephemPtr_.delta_tau_n; 
//                 gnss_raw.tgd = 0;

//                 gnss_raw.sat_pos_x = sat_ecef[0];
//                 gnss_raw.sat_pos_y = sat_ecef[1];
//                 gnss_raw.sat_pos_z = sat_ecef[2];
//                 gnss_raw.vel_x = sat_vel[0];
//                 gnss_raw.vel_y = sat_vel[1];
//                 gnss_raw.vel_z = sat_vel[2];
//                 sat_azel(receiver_pos_ecef, sat_ecef, azel);

//                 gnss_raw.azimuth = azel[0]*180/PI;
//                 gnss_raw.elevation = azel[1]*180/PI;
//                 gnss_raw_mf.azimuth = gnss_raw.azimuth;
//                 gnss_raw_mf.elevation = gnss_raw.elevation;
//                 azel_[0 + s_i*2]=azel[0];
//                 azel_[1 + s_i*2]=azel[1];

//                 ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);

//             }
//             else
//             {
//                 ROS_WARN("Satellite ID %d not found in position or velocity maps.", sat_id);
//                 continue; // Skip this satellite if not found in maps
//             }



//             double dion, dtrp, vmeas, vion, vtrp, rr[3], pos[3], e[3], P, lam_L1;
//             /* psudorange with code bias correction */
//             // if ((P = prange(obs + s_i, nav, azel_ + s_i * 2, 2, &opt_, &vmeas)) == 0.0)  //revise
//             //     continue;

//             /* ionospheric corrections */
//             for (int i = 0; i < 3; i++)
//                 rr[i] = lla_pvt[i];
//             ecef2pos(rr, pos);
//             // if (!ionocorr(obs[s_i].time, nav, obs[s_i].sat, pos, azel_ + s_i * 2,
//             //               opt->ionoopt, &dion, &vion))
//             //     continue;
            
//             // *ion=ionmodel(time,nav->ion_gps,pos,azel); //revise
//             // *var=SQR(*ion*ERR_BRDCI);
            
//             ::gtime_t time_cur = gpst2time(current_week, current_tow);
//             dion=ionmodel(time_cur,iono_msg.data.data(),pos,azel_ + s_i * 2); //revise careful
//             vion=(dion*0.5)*(dion*0.5);

//             // ROS_INFO("ionmodel finish.");
//             // printf("dion-> WLS %f \n", dion);
//             // printf("dtrp %f \n", dtrp);

//             /* GPS-L1 -> L1/B1 */
//             double lam_nav=299792458.0/msg->meas[s_i].freqs[0];
//             if ((lam_nav) > 0.0)
//             {
//                 dion *= (lam_nav / msg->meas[s_i].cp[0])*(lam_nav / msg->meas[s_i].cp[0]);
//             }

//             /* tropospheric corrections
//              * pose of receievr.
//              */
//             // if (!tropcorr(obs[s_i].time, nav, pos, azel_ + s_i * 2,
//             //               opt->tropopt, &dtrp, &vtrp))
//             // {
//             //     continue;
//             // }

//             dtrp=tropmodel(time_cur,pos,azel_,REL_HUMI); //REL_HUMI=0.7
//             vtrp=(ERR_SAAS/(sin(azel_[1])+0.1))*(ERR_SAAS/(sin(azel_[1])+0.1));  //ERR_SAAS=0.3

//             // ROS_INFO("tropmodel finish.");
//             // printf("dtrp %f \n", dtrp);
//             gnss_raw.err_tropo = dtrp;
//             gnss_raw.err_iono = dion;


//             gnss_raw_mf.err_tropo = dtrp;
//             gnss_raw_mf.sat_clk_err = gnss_raw.sat_clk_err;  
//             gnss_raw_mf.ttx = 0;
//             gnss_raw_mf.dt = gnss_raw.dt;
//             gnss_raw_mf.ddt = gnss_raw.ddt;
//             gnss_raw_mf.tgd = 0;
//             gnss_raw_mf.sat_pos_x = gnss_raw.sat_pos_x;
//             gnss_raw_mf.sat_pos_y = gnss_raw.sat_pos_y;
//             gnss_raw_mf.sat_pos_z = gnss_raw.sat_pos_z;
//             gnss_raw_mf.vel_x = gnss_raw.vel_x;
//             gnss_raw_mf.vel_y = gnss_raw.vel_y;
//             gnss_raw_mf.vel_z = gnss_raw.vel_z;

//             /* get pr*/
//             double pr = 0;
//             for (int j = 0; j < NFREQ; j++)
//                 if ((pr = msg->meas[s_i].psr[j]) != 0.0)
//                     break;
//             gnss_raw.pseudorange = msg->meas[s_i].psr[0]; //revise
//             // gnss_raw.pseudorange = pr;
//             // gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;
//             /* remove the satellite clock bias, atmosphere error here */
//             //gnss_raw.pseudorange = P + gnss_raw.sat_clk_err - dion - dtrp;  //revise

//             gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;  

//             gnss_raw.raw_pseudorange = msg->meas[s_i].psr[0];
//             gnss_raw.carrier_phase = msg->meas[s_i].cp[0];
//             gnss_raw.doppler = msg->meas[s_i].dopp[0];
//             gnss_raw.LLI = msg->meas[s_i].LLI[0];
//             gnss_raw.slip = (msg->meas[s_i].LLI[0] & 3) > 0 ? 1 : 0;
//             gnss_raw.lamda = lam_nav;

//             int sys = satsys(msg->meas[s_i].sat, NULL);
//             gnss_raw_mf.constellation = sys;

//             for (int j = 0; j < NFREQ + NEXOBS; j++)
//             {
                
//                 // gnss_raw_mf.lamda.push_back(nav->lam[obs[s_i].sat - 1][j]);
//                 // gnss_raw_mf.err_iono.push_back(dion * SQR(nav->lam[obs[s_i].sat - 1][j] / nav->lam[obs[s_i].sat - 1][0]));
//                 // // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
//                 // gnss_raw_mf.pseudorange.push_back(1.0);
//                 // //            gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
//                 // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
//                 // gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
//                 // gnss_raw_mf.doppler.push_back(obs[s_i].D[j]);
//                 // gnss_raw_mf.snr.push_back(obs[s_i].SNR[j] * 0.25);


//                 double lam_LL = 299792458.0/msg->meas[s_i].freqs[j];
//                 gnss_raw_mf.lamda.push_back(lam_LL);
//                 gnss_raw_mf.err_iono.push_back(dion * (lam_LL / lam_nav)*(lam_LL / lam_nav));
//                 gnss_raw_mf.pseudorange.push_back(1.0);
//                 gnss_raw_mf.raw_pseudorange.push_back(msg->meas[s_i].psr[j]);
//                 gnss_raw_mf.raw_carrier_phase.push_back(msg->meas[s_i].cp[j]);
//                 gnss_raw_mf.doppler.push_back(msg->meas[s_i].dopp[j]);
//                 gnss_raw_mf.snr.push_back(msg->meas[s_i].CN0[j] * 0.25);


//             }

//             if (gnss_raw.elevation > 15) // 15
//             {
//                 gnss_data.GNSS_Raws.push_back(gnss_raw);
//                 gnss_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
//             }

// #if 1 // debug satellite information
//             if (sys == SYS_GPS)
//             {
//                 GPS_cnt++;
//             }
//             else if (sys == SYS_CMP)
//             {
//                 haveOneBeiDou = true;
//                 CMP_cnt++;
//             }
//             else if (sys == SYS_GAL)
//             {
//                 GAL_cnt++;
//             }
//             else if (sys == SYS_GLO)
//             {
//                 GLO_cnt++;
//             }
// #endif
//         }

//         // Ensure publisher is valid before publishing
//         if (pub_gnss_raw)
//         {
//             if (fabs(current_tow - lastGNSSTime) > 0.0)
//             {
//                 pub_gnss_raw.publish(gnss_data);
//                 lastGNSSTime = current_tow;
//             }
//             ROS_ERROR("publish gnss_data.");
//         }
//         else
//         {
//             ROS_ERROR("Publisher is invalid, cannot publish gnss_data.");
//         }

//         // if (fabs(current_tow - lastGNSSTime) > 0.0)
//         // {
//         //     pub_gnss_raw.publish(gnss_data);
//         //     lastGNSSTime = current_tow;
//         // }
//         // ROS_ERROR("publish gnss_data.");
//     }







/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
* notes  : assuming sbas-gps, galileo-gps, qzss-gps, compass-gps time offset and
*          receiver bias are negligible (only involving glonass-gps time offset
*          and receiver bias)
*-----------------------------------------------------------------------------*/

//     void measCallback(const gnss_comm::GnssMeasMsg::ConstPtr &msg)
//     {

//         int n=msg->meas.size();

//         double *rs, *dts, *var, *azel_, *resp;
//         int i, stat, vsat[MAXOBS] = {0}, svh[MAXOBS];

//         // sol->stat = SOLQ_NONE;

//         //trace(3, "pntpos  : tobs=%s n=%d\n", time_str(obs[0].time, 3), n);

//         rs = mat(6, n);
//         dts = mat(2, n);
//         var = mat(1, n);
//         azel_ = zeros(2, n);
//         resp = mat(1, n);
//         nlosExclusion::GNSS_Raw_Array gnss_data; // revise

//         if (n <= 0)
//         {
//             std::cerr << "Error: no measurements available. Exiting callback." << std::endl;
//             return ;
//         }

//         int current_week = msg->meas[0].time.week;
//         double current_tow = msg->meas[0].time.tow;
//         double epoch_time[100];
//         ::gtime_t current_time = gpst2time(msg->meas[0].time.week, msg->meas[0].time.tow);
//         time2epoch(current_time, epoch_time);


//         // satazel();

//         /* estimate receiver position with pseudorange by WLS and Eigen */
//         bool haveOneBeiDou = false;
//         int CMP_cnt = 0, GPS_cnt = 0, GAL_cnt = 0, GLO_cnt = 0;

//         //std::map<int, std::pair<double, double>> azimuth_elevation_map;
//         computeAzimuth(azimuth_elevation_map);

//         for (int s_i = 0; s_i < n; s_i++)
//         {
//             nlosExclusion::GNSS_Raw gnss_raw;
//             gnss_raw.GNSS_time = current_tow;
//             gnss_raw.prE3dMA = current_week;
//             gnss_raw.GNSS_week = current_week;
//             gnss_raw.total_sv = float(n);
//             gnss_raw.prn_satellites_index = float(msg->meas[s_i].sat);

//             nlosExclusion::GNSS_Raw_mf gnss_raw_mf;
//             gnss_raw_mf.GNSS_week = current_week;
//             gnss_raw_mf.GNSS_time = current_tow;
//             gnss_raw_mf.total_sv = float(n);
//             gnss_raw_mf.prn_satellites_index = float(msg->meas[s_i].sat);

//             //NFREQ=3
//             /* get snr*/
//             double snr = 0;
//             for (int j = 0, snr = 0.0; j < NFREQ; j++)
//                 if ((snr = msg->meas[s_i].CN0[j]) != 0.0)
//                     break;
//             gnss_raw.snr = msg->meas[s_i].CN0[0] * 0.25;
//             // gnss_raw.snr = snr;

//             // 更新方位角和高度角
//             int sat_id = msg->meas[s_i].sat;
//             ROS_INFO("sat star.");
//             if (azimuth_elevation_map.find(sat_id) != azimuth_elevation_map.end())
//             {
//                 ROS_INFO("Satellite ID %d found in azimuth_elevation_map", sat_id);
//                 gnss_raw.azimuth = azimuth_elevation_map.at(sat_id).first;
//                 gnss_raw.elevation = azimuth_elevation_map.at(sat_id).second;
//                 gnss_raw_mf.azimuth = azimuth_elevation_map.at(sat_id).first;
//                 gnss_raw_mf.elevation = azimuth_elevation_map.at(sat_id).second;
//                 azel_[0 + s_i*2]=gnss_raw.azimuth*PI/180.0;
//                 azel_[1 + s_i*2]=gnss_raw.elevation*PI/180.0;
//             }
//             double dion, dtrp, vmeas, vion, vtrp, rr[3], pos[3], e[3], P, lam_L1;
//             /* psudorange with code bias correction */
//             // if ((P = prange(obs + s_i, nav, azel_ + s_i * 2, 2, &opt_, &vmeas)) == 0.0)  //revise
//             //     continue;

//             /* ionospheric corrections */
//             for (int i = 0; i < 3; i++)
//                 rr[i] = lla_pvt[i];
//             ecef2pos(rr, pos);
//             // if (!ionocorr(obs[s_i].time, nav, obs[s_i].sat, pos, azel_ + s_i * 2,
//             //               opt->ionoopt, &dion, &vion))
//             //     continue;
            
//             // *ion=ionmodel(time,nav->ion_gps,pos,azel); //revise
//             // *var=SQR(*ion*ERR_BRDCI);
            
//             ::gtime_t time_cur = gpst2time(current_week, current_tow);
//             dion=ionmodel(time_cur,iono_msg.data.data(),pos,azel_ + s_i * 2); //revise careful
//             vion=(dion*0.5)*(dion*0.5);

//             // ROS_INFO("ionmodel finish.");
//             // printf("dion-> WLS %f \n", dion);
//             // printf("dtrp %f \n", dtrp);

//             /* GPS-L1 -> L1/B1 */
//             double lam_nav=299792458.0/msg->meas[s_i].freqs[0];
//             if ((lam_nav) > 0.0)
//             {
//                 dion *= (lam_nav / msg->meas[s_i].cp[0])*(lam_nav / msg->meas[s_i].cp[0]);
//             }

//             /* tropospheric corrections
//              * pose of receievr.
//              */
//             // if (!tropcorr(obs[s_i].time, nav, pos, azel_ + s_i * 2,
//             //               opt->tropopt, &dtrp, &vtrp))
//             // {
//             //     continue;
//             // }

//             dtrp=tropmodel(time_cur,pos,azel_,REL_HUMI); //REL_HUMI=0.7
//             vtrp=(ERR_SAAS/(sin(azel_[1])+0.1))*(ERR_SAAS/(sin(azel_[1])+0.1));  //ERR_SAAS=0.3

//             // ROS_INFO("tropmodel finish.");
//             // printf("dtrp %f \n", dtrp);
//             gnss_raw.err_tropo = dtrp;
//             gnss_raw.err_iono = dion;

//             // gnss_raw.sat_clk_err = dts[0 + s_i * 2] * CLIGHT;  //CLIGHT=299792458.0  revise
//             // gnss_raw.ttx = 0;
//             // gnss_raw.dt = dts[0 + s_i * 2] * CLIGHT;  //CLIGHT=299792458.0  revise
//             // gnss_raw.ddt = dts[1 + s_i * 2] * CLIGHT;  //CLIGHT=299792458.0  revise

//             if (satellite_positions.find(sat_id) != satellite_positions.end())
//             {

//                 gnss_raw.sat_clk_err = satellite_dt.at(sat_id)(0); // CLIGHT=299792458.0  revise
//                 gnss_raw.ttx = 0;
//                 gnss_raw.dt = satellite_dt.at(sat_id)(1);  // CLIGHT=299792458.0  revise
//                 gnss_raw.ddt = satellite_dt.at(sat_id)(2); // CLIGHT=299792458.0  revise
//                 gnss_raw.tgd = 0;

//                 gnss_raw.sat_pos_x = satellite_positions.at(sat_id)(0);
//                 gnss_raw.sat_pos_y = satellite_positions.at(sat_id)(1);
//                 gnss_raw.sat_pos_z = satellite_positions.at(sat_id)(2);
//                 gnss_raw.vel_x = satellite_vel.at(sat_id)(0);
//                 gnss_raw.vel_y = satellite_vel.at(sat_id)(1);
//                 gnss_raw.vel_z = satellite_vel.at(sat_id)(2);
//                 ROS_INFO("Satellite ID %d found in satellite_positions.", sat_id);
//             }
//             else
//             {
//                 ROS_WARN("Satellite ID %d not found in position or velocity maps.", sat_id);
//                 continue; // Skip this satellite if not found in maps
//             }
//             // gnss_raw.sat_pos_x = satellite_positions.at(sat_id).first;//satellite_positions.at(sat_id).first
//             // gnss_raw.sat_pos_y = satellite_positions.at(sat_id).second;
//             // gnss_raw.sat_pos_z = satellite_positions.at(sat_id).third;
//             // gnss_raw.vel_x = satellite_vel.at(sat_id).first;
//             // gnss_raw.vel_y = satellite_vel.at(sat_id).second;
//             // gnss_raw.vel_z = satellite_vel.at(sat_id).third;

//             gnss_raw_mf.err_tropo = dtrp;
//             gnss_raw_mf.sat_clk_err = gnss_raw.sat_clk_err;  
//             gnss_raw_mf.ttx = 0;
//             gnss_raw_mf.dt = gnss_raw.dt;
//             gnss_raw_mf.ddt = gnss_raw.ddt;
//             gnss_raw_mf.tgd = 0;
//             gnss_raw_mf.sat_pos_x = gnss_raw.sat_pos_x;
//             gnss_raw_mf.sat_pos_y = gnss_raw.sat_pos_y;
//             gnss_raw_mf.sat_pos_z = gnss_raw.sat_pos_z;
//             gnss_raw_mf.vel_x = gnss_raw.vel_x;
//             gnss_raw_mf.vel_y = gnss_raw.vel_y;
//             gnss_raw_mf.vel_z = gnss_raw.vel_z;

//             /* get pr*/
//             double pr = 0;
//             for (int j = 0; j < NFREQ; j++)
//                 if ((pr = msg->meas[s_i].psr[j]) != 0.0)
//                     break;
//             gnss_raw.pseudorange = msg->meas[s_i].psr[0]; //revise
//             // gnss_raw.pseudorange = pr;
//             // gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;
//             /* remove the satellite clock bias, atmosphere error here */
//             //gnss_raw.pseudorange = P + gnss_raw.sat_clk_err - dion - dtrp;  //revise

//             gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;  

//             gnss_raw.raw_pseudorange = msg->meas[s_i].psr[0];
//             gnss_raw.carrier_phase = msg->meas[s_i].cp[0];
//             gnss_raw.doppler = msg->meas[s_i].dopp[0];
//             gnss_raw.LLI = msg->meas[s_i].LLI[0];
//             gnss_raw.slip = (msg->meas[s_i].LLI[0] & 3) > 0 ? 1 : 0;
//             gnss_raw.lamda = lam_nav;

//             int sys = satsys(msg->meas[s_i].sat, NULL);
//             gnss_raw_mf.constellation = sys;

//             for (int j = 0; j < NFREQ + NEXOBS; j++)
//             {
                
//                 // gnss_raw_mf.lamda.push_back(nav->lam[obs[s_i].sat - 1][j]);
//                 // gnss_raw_mf.err_iono.push_back(dion * SQR(nav->lam[obs[s_i].sat - 1][j] / nav->lam[obs[s_i].sat - 1][0]));
//                 // // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
//                 // gnss_raw_mf.pseudorange.push_back(1.0);
//                 // //            gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
//                 // gnss_raw_mf.raw_pseudorange.push_back(obs[s_i].P[j]);
//                 // gnss_raw_mf.raw_carrier_phase.push_back(obs[s_i].L[j]);
//                 // gnss_raw_mf.doppler.push_back(obs[s_i].D[j]);
//                 // gnss_raw_mf.snr.push_back(obs[s_i].SNR[j] * 0.25);


//                 double lam_LL = 299792458.0/msg->meas[s_i].freqs[j];
//                 gnss_raw_mf.lamda.push_back(lam_LL);
//                 gnss_raw_mf.err_iono.push_back(dion * (lam_LL / lam_nav)*(lam_LL / lam_nav));
//                 gnss_raw_mf.pseudorange.push_back(1.0);
//                 gnss_raw_mf.raw_pseudorange.push_back(msg->meas[s_i].psr[j]);
//                 gnss_raw_mf.raw_carrier_phase.push_back(msg->meas[s_i].cp[j]);
//                 gnss_raw_mf.doppler.push_back(msg->meas[s_i].dopp[j]);
//                 gnss_raw_mf.snr.push_back(msg->meas[s_i].CN0[j] * 0.25);


//             }

//             if (gnss_raw.elevation > 15) // 15
//             {
//                 gnss_data.GNSS_Raws.push_back(gnss_raw);
//                 gnss_data.GNSS_Raws_mf.push_back(gnss_raw_mf);
//             }

// #if 1 // debug satellite information
//             if (sys == SYS_GPS)
//             {
//                 GPS_cnt++;
//             }
//             else if (sys == SYS_CMP)
//             {
//                 haveOneBeiDou = true;
//                 CMP_cnt++;
//             }
//             else if (sys == SYS_GAL)
//             {
//                 GAL_cnt++;
//             }
//             else if (sys == SYS_GLO)
//             {
//                 GLO_cnt++;
//             }
// #endif
//         }

//         // Ensure publisher is valid before publishing
//         if (pub_gnss_raw)
//         {
//             if (fabs(current_tow - lastGNSSTime) > 0.0)
//             {
//                 pub_gnss_raw.publish(gnss_data);
//                 lastGNSSTime = current_tow;
//             }
//             ROS_ERROR("publish gnss_data.");
//         }
//         else
//         {
//             ROS_ERROR("Publisher is invalid, cannot publish gnss_data.");
//         }

//         // if (fabs(current_tow - lastGNSSTime) > 0.0)
//         // {
//         //     pub_gnss_raw.publish(gnss_data);
//         //     lastGNSSTime = current_tow;
//         // }
//         // ROS_ERROR("publish gnss_data.");
//     }


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ublox_ros");
    ROS_INFO("Ublox ROS Node Started.");

    UbloxRos ublox_ros;

    ros::spin();

    return 0;
}

