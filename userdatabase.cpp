#pragma execution_character_set("utf-8")
/*******************************************
* @className
* @brief         Data Base
*                v1.0
* @author        cql
* @date          2019-01-17
********************************************/

#include "userdatabase.h"
#include <QDebug>
#define DATA_BASE_NAME ("mysql.db")//数据库文件名
QSqlDatabase userDataBasePri::database;

userDataBasePri::userDataBasePri()
{
    if (QSqlDatabase::contains("cql_sqllite")) {//连接数据库
        database = QSqlDatabase::database("cql_sqllite");
    } else {
        database = QSqlDatabase::addDatabase("QSQLITE","cql_sqllite");
    }
    database.setDatabaseName(DATA_BASE_NAME);//设置数据库文件名
    database.setUserName("root");//用户
    database.setPassword("nopassword");//密码-无密码

    if (! database.open()){//打开数据库
        qCritical() << database.lastError().text();
        return;
    }

    QSqlQuery query(database);
    //管理员用户
    bool ret = query.exec(QString::fromLocal8Bit("create table user_information(name varchar(50) primary key,passwd varchar(20),sex varchar(10),age varchar(20))"));
    query.exec(QString::fromLocal8Bit("insert into user_information values('admin','passwd','男','22')"));

    //用户
    ret = query.exec(QString::fromLocal8Bit("create table camera_info(id varchar(50) primary key,url varchar(500))"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag1','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag2','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag3','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag4','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag5','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag6','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag7','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag8','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag9','0')"));

    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag10','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag11','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag12','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag13','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag14','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag15','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag16','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag17','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag18','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag19','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag20','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag21','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag22','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag23','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag24','0')"));
    query.exec(QString::fromLocal8Bit("insert into camera_info values('flag25','0')"));


    database.close();
    qDebug()<<"ddddddddd"<<ret;
}

userDataBasePri::~userDataBasePri()
{

}

QVector<QVector<QString>> userDataBasePri::queryAll(QString tablename)
{
    if (! userDataBasePri::database.open()){//打开数据库
        qCritical() << userDataBasePri::database.lastError().text();
    }
    QSqlQuery query(userDataBasePri::database);
    QString statement = QString("SELECT * FROM %1").arg(tablename);
    query.exec(statement);
    QVector<QVector<QString>> retvector;
    QVector<QString> vectorstr;
    int midsize = 0;
    while(query.next())
    {
        vectorstr.clear();
        midsize = query.size();
        for(int i = 0; i < 40;i++ ){
            vectorstr.append(query.value(i).toByteArray());
        }
        retvector.append(vectorstr);
    }
    userDataBasePri::database.close();
    return retvector;
}
