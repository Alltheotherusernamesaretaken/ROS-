#include "MotorManager.h"

MotorManager::MotorManager(
    std::vector<MotorInterfaceABC*> motors,
    std::map<std::string, int> motor_map
    )
    {
    motors = motors;
    motor_map = motor_map;
    }

int MotorManager::getMotorIdx(std::string name, int* idx){
    if (motor_map.find(name) != motor_map.end()){
        *idx = motor_map[name];
        return 0;
    }
    *idx = -1;
    return -1;
    }

int MotorManager::setPos(double stpt, int idx){
    start();
    return motors[idx]->setPos(stpt);
    end();
    }

int MotorManager::setVel(double stpt, int idx){
    start();
    return motors[idx]->setVel(stpt);
    end();
    }


int MotorManager::getPos(double* pos, int idx){
        start();
        return motors[idx]->getPos(pos);
        end();
    }


int MotorManager::getVel(double* vel, int idx){
        start();
        return motors[idx]->getVel(vel);
        end();
    }


int MotorManager::setPos(double stpt, std::string mot){
    int* idx;
    int err = getMotorIdx(mot, idx);
    if (!err){
        setPos(stpt, *idx);
    }
    return err;
    }

int MotorManager::setPos(double* stpts, std::string* mots, int cnt){
    int err = 0;
    for(int i=0; i<cnt; i++){
        err = err + setPos(stpts[i], mots[i]);
    }
    return err;
    }

int MotorManager::getPos(std::string mot, double* pos){
    int* idx;
    int err = getMotorIdx(mot, idx);
    if (!err){
        return getPos(pos, *idx);
    }
    return err;
    }

int MotorManager::getPos(std::string* mot, int cnt, double* pos){
    int err = 0;
    for(int i=0; i<cnt; i++){
        err = err + getPos(mot[i], &(pos[i]));
    }
    return err;
    }


int MotorManager::setVel(double stpt, std::string mot){
    int* idx;
    int err = getMotorIdx(mot, idx);
    if (!err){
        setVel(stpt, *idx);
    }
    return err;
    }

int MotorManager::setVel(double* stpts, std::string* mots, int cnt){
    int err = 0;
    for(int i=0; i<cnt; i++){
        err = err + setVel(stpts[i], mots[i]);
    }
    return err;
    }

int MotorManager::getVel(std::string mot, double* vel){
    int* idx;
    int err = getMotorIdx(mot, idx);
    if (!err){
        return getVel(vel, *idx);
    }
    return err;
    }

int MotorManager::getVel(std::string* mot, int cnt, double* vel){
    int err = 0;
    for(int i=0; i<cnt; i++){
        err = err + getVel(mot[i], &(vel[i]));
    }
    return err;
    }

int MotorManager::update(){
    int err = 0;
    start();
    for(
        auto it = motors.begin()
        ; it != motors.end()
        ; ++it
        )
    {
        err = err + (*it)->update();
    }
    return err;
    end();
    }