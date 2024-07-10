
/*!
  @file    userDataBase.hpp
  @author  chen
  @date    2018/9
  @brief   The file Declare database interface
  @https     https://blog.csdn.net/cqltbe131421
  @verbatim
          <author>    <time>   <version>  <desc>
         chen 2018/9   0.1.0     build this module
  @endverbatim
*/

#ifndef USERDATABASE_H
#define USERDATABASE_H
#include <QTextCodec>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QTime>
#include <QSqlError>
#include <QtDebug>
#include <QSqlDriver>
#include <QSqlRecord>


/*!
  @addtogroup  userDataInfo Patient information class
  @{
*/

class userDataBasePri
{
public:
    userDataBasePri();//构造函数中创建数据库文件
    ~userDataBasePri();

   static QVector<QVector<QString>> queryAll(QString tablename);

public:
    static  QSqlDatabase database;//数据库
};

#endif // USERDATABASE_H
