#define EIGEN_STACK_ALLOCATION_LIMIT 0
#include "realtime.h"
#include <iostream>
#include <chrono>
#include <time.h>
#include <atomic>

#include <thread>
#include <dwbc/dwbc.h>

#define LOOP_CNT 10000

class tlocker
{
public:
    tlocker()
    {
        counter = 0;
    }
    void producer_lock()
    {
        while (lock.test_and_set(std::memory_order_acquire))
        {
        }
    }

    void producer_ready()
    {
        counter++;
        lock.clear(std::memory_order_release);
    }

    void consumer_wait()
    {
        while (true)
        {
            if (counter > 0)
            {
                while (lock.test_and_set(std::memory_order_acquire))
                {
                }

                counter--;
                break;
            }
            else
            {
                asm("pause");
            }
        }
    }

    void consumer_done()
    {
        lock.clear(std::memory_order_release);
    }

private:
    std::atomic_flag lock = ATOMIC_FLAG_INIT;
    std::atomic_int8_t counter;
};

tlocker tlock_;
tlocker tlock2_;

std::atomic_int32_t aint_;
DWBC::RobotData rd_gl_;
VectorXd torque_command_gl_;

void *realtime_thread1(void *data)
{
    DWBC::RobotData rd_;

    std::string urdf_path = "/home/dyros/libdwbc/test/dyros_tocabi.urdf";

    rd_.InitModelData(urdf_path.c_str(), true, false);
    rd_gl_.InitModelData(urdf_path.c_str(), true, false);

    int left_foot_id = 6;
    int right_foot_id = 12;
    rd_.AddContactConstraint(left_foot_id, DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    VectorXd q, qdot, qddot;

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    VectorXd qmod;
    qmod.setZero(rd_.model_.q_size);

    VectorXd qr;
    qr.setZero(rd_.model_.q_size);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    aint_ = 0;

    int thread_cnt_ = 0;
    VectorXd torque_command;

    printf("test uk\n");
    rd_.UpdateKinematics(qr, qdot, qddot);

    printf("start thread1\n");

    long t_total = 0;

    int t_min = 100000;
    int t_max = 0;

    while (thread_cnt_ < LOOP_CNT)
    {
        sync_period(&ts, 1000000);

        auto t1 = std::chrono::steady_clock::now();

        qmod.setRandom();
        qr = q + qmod * 0.01;
        qr.segment(3, 3).setZero();
        qr(rd_.system_dof_) = 1;

        rd_.UpdateKinematics(qr, qdot, qddot);

        tlock_.producer_lock();

        rd_.CopyKinematicsData(rd_gl_);

        tlock_.producer_ready();

        tlock2_.consumer_wait();

        torque_command = torque_command_gl_;

        tlock2_.consumer_done();

        auto t2 = std::chrono::steady_clock::now();

        int t_dur = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        t_total += t_dur;

        if (t_min > t_dur)
            t_min = t_dur;
        if (t_max < t_dur)
            t_max = t_dur;

        thread_cnt_++;
        // printf("Thread1 : %d\n", thread_cnt_);
    }

    printf("thread1 counnt : %d total : %d min %d max %d\n", thread_cnt_, (int)(t_total / thread_cnt_), t_min, t_max);

    return NULL;
}

void *realtime_thread2(void *data)
{
    printf("hello world2\n");

    DWBC::RobotData rd_;
    std::string urdf_path = "/home/dyros/libdwbc/test/dyros_tocabi.urdf";
    rd_.InitModelData(urdf_path.c_str(), true, false);
    VectorXd fstar;
    fstar.setZero(6);
    fstar(0) = 0.1;
    fstar(1) = 4.0;
    fstar(2) = 0.1;

    fstar(3) = 0.1;
    fstar(4) = -0.1;
    fstar(5) = 0.1;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    int thread_cnt_ = 0;

    VectorXd torque_command_;

    bool init_ = true;

    long t_total = 0;

    while (thread_cnt_ < LOOP_CNT)
    {
        tlock_.consumer_wait();

        rd_gl_.CopyKinematicsData(rd_);

        tlock_.consumer_done();

        auto t1 = std::chrono::steady_clock::now();

        try
        {

            rd_.SetContact(true, true);

            if (init_)
            {
                rd_.AddTaskSpace(DWBC::TASK_LINK_6D, 0, Vector3d::Zero());
                rd_.AddTaskSpace(DWBC::TASK_LINK_ROTATION, 15, Vector3d::Zero());
            }

            rd_.SetTaskSpace(0, fstar);
            rd_.SetTaskSpace(1, fstar.segment(3, 3));

            rd_.CalcGravCompensation(); // Calulate Gravity Compensation
            rd_.CalcTaskControlTorque(init_);

            rd_.CalcContactRedistribute(init_);
        }
        catch (const std::bad_alloc &e)
        {
            std::cout << e.what() << std::endl;
        }

        torque_command_ = rd_.torque_contact_ + rd_.torque_grav_ + rd_.torque_task_;

        init_ = false;

        auto tdur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t1).count();

        t_total += tdur;

        tlock2_.producer_lock();

        torque_command_gl_ = torque_command_;

        tlock2_.producer_ready();

        thread_cnt_++;
        // printf("Thread2 : %d\n", thread_cnt_);
    }

    printf("thread2 counnt : %d, aint : %d avg : %d\n", thread_cnt_, (int)aint_, (int)(t_total / thread_cnt_));
    return NULL;
}

int main(void)
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    pthread_t thread;
    pthread_t thread2;

    pthread_attr_t attr1, attr2;

    sched_param param1_;
    sched_param param2_;

    printf("creating rt thread\n");
    // pthread_create_rt(&thread, &attr1, &param1_, 95, realtime_thread1, NULL);

    // pthread_create_rt(&thread2, &attr2, &param2_, 94, realtime_thread2, NULL);

    // pthread_attr_t attr;

    // pthread_attr_init(&attr1);
    // // pthread_attr_setstacksize(&attr1, PTHREAD_STACK_MIN);
    // pthread_attr_setschedpolicy(&attr1, SCHED_FIFO);
    // param1_.sched_priority = 95;

    // pthread_attr_setschedparam(&attr1, &param1_);
    // pthread_attr_setinheritsched(&attr1, PTHREAD_EXPLICIT_SCHED);

    // pthread_attr_init(&attr2);
    // pthread_attr_setschedpolicy(&attr2, SCHED_FIFO);
    // param2_.sched_priority = 94;

    // pthread_attr_setschedparam(&attr2, &param2_);
    // pthread_attr_setinheritsched(&attr2, PTHREAD_EXPLICIT_SCHED);

    pthread_create(&thread, NULL, &realtime_thread1, NULL);
    pthread_create(&thread2, NULL, &realtime_thread2, NULL);

    pthread_join(thread, NULL);
    pthread_join(thread2, NULL);

    // std::thread t1_;
    // std::thread t2_;

    // t1_ = std::thread(realtime_thread1, (void *)NULL);
    // t2_ = std::thread(realtime_thread2, (void *)NULL);

    // t1_.join();
    // t2_.join();

    // std::thread

    return 0;
}
