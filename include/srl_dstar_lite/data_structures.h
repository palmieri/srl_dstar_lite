///<  @brief Struct that descrbes a human point
typedef struct HumanPoint
{

  // pose of a human being (agent)
  double x;
  double y;
  double z;
  // agent id and type
  int id;
  int type;

  // cell_width and cell_height used as agent's sizes
  double cell_width;
  double cell_height;

  HumanPoint()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    cell_width = 0.3;
    cell_height = 0.4;
    id = 0;
    type = 0;
  }
  HumanPoint(double xx, double yy) : x(xx), y(yy)
  {
    z = 0.0;
    cell_width = 0.3;
    cell_height = 0.4;
    id = 0;
    type = 0;
  }
  HumanPoint(double xx, double yy, double zz, double ii, double tt) : x(xx), y(yy), z(zz), id(ii), type(tt)
  {
    cell_width = 0.3;
    cell_height = 0.4;
  }
  HumanPoint(double* p)
  {
    x = p[0];
    y = p[1];
    z = p[2];
    id = p[3];
    type = p[4];
    cell_width = 0.3;
    cell_height = 0.4;
  }

  HumanPoint& operator=(HumanPoint const& copy)
  {
    x = copy.x;
    y = copy.y;
    z = copy.z;
    cell_width = copy.cell_width;
    cell_height = copy.cell_height;
    id = copy.id;
    type = copy.type;

    return *this;
  }

} Thuman;

///<  @brief Struct that descrbes an Obstacle point
typedef struct ObstaclePoint
{

  double x;
  double y;
  double z;
  double cell_width;
  double cell_height;

  ObstaclePoint()
  {
    x = 0.0;
    y = 0.0;
    z = 0.0;
    cell_width = 0;
    cell_height = 0;
  }
  ObstaclePoint(double xx, double yy) : x(xx), y(yy)
  {
    z = 0.0;
    cell_width = 0;
    cell_height = 0;
  }
  ObstaclePoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz)
  {
    cell_width = 0;
    cell_height = 0;
  }
  ObstaclePoint(double xx, double yy, double zz, double cw, double ch)
      : x(xx), y(yy), z(zz), cell_width(cw), cell_height(ch)
  {
  }
  ObstaclePoint(double* p)
  {
    x = p[0];
    y = p[1];
    z = p[2];
    cell_width = p[3];
    cell_height = p[4];
  }

  ObstaclePoint& operator=(ObstaclePoint const& copy)
  {
    x = copy.x;
    y = copy.y;
    z = copy.z;
    cell_width = copy.cell_width;
    cell_height = copy.cell_height;

    return *this;
  }

} Tobstacle;
