#ifndef MONGO_HELPER_H
#define MONGO_HELPER_H

#include <cstdint>
#include <iostream>
#include <vector>
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/stdx.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/instance.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/array.hpp>

using bsoncxx::builder::stream::close_array;
using bsoncxx::builder::stream::close_document;
using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;
using bsoncxx::builder::stream::open_array;
using bsoncxx::builder::stream::open_document;

class MongoHelper
{
  private:
    // Singleton Design
    static MongoHelper *instance;
    int data;

    MongoHelper()
    {
       data = 0;
    }

  public:
    static MongoHelper *getInstance()
    {
      if (!instance)
        instance = new MongoHelper;
      return instance;
    }

    int getData()
    {
      return this -> data;
    }

    void setData(int data)
    {
      this -> data = data;
    }
};
#endif
