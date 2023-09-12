#include "polygon.h"

Polygon::Polygon(const std::vector<cv::Point2d>& points)
{
	this->points = points;
}

Polygon::~Polygon()
{
}

bool Polygon::JudgePointInside(cv::Point2d p)
{
	if (find == 0) {
		minx = MAX_D;
		maxx = MIN_D;
		miny = MAX_D;
		maxy = MIN_D;
		for (int i = 0; i < points.size(); i++) {
			minx = std::min(points[i].x, minx);
			maxx = std::max(points[i].x, maxx);
			miny = std::min(points[i].y, miny);
			maxy = std::max(points[i].y, maxy);
		}
		find = 1;
	}
	if (p.x < minx || p.x > maxx || p.y < miny || p.y > maxy) return false;
	int i, j, c = 0;
	for (i = 0, j = points.size() - 1; i < points.size(); j = i++) {
		if (((points[i].y > p.y) != (points[j].y > p.y)))
			if(p.x < (points[j].x - points[i].x) * (p.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
				c = !c;
	}
	return c;
}

void Polygon::Draw(int index)
{
	cv::Mat img(3000, 3000, CV_8UC3);
	for (int i = 0; i < this->points.size() - 1; i++) {
		cv::line(img, cv::Point(this->points[i].x * 10 + 1500, this->points[i].y * 10 + 1500), cv::Point(this->points[i + 1].x * 10 + 1500, this->points[i + 1].y * 10 + 1500), cv::Scalar(0, 255, 255), 3);
	}
	cv::line(img, cv::Point(this->points[0].x * 10 + 1500, this->points[0].y * 10 + 1500), cv::Point(this->points[this->points.size() - 1].x * 10 + 1500, this->points[this->points.size() - 1].y * 10 + 1500), cv::Scalar(0, 255, 255), 3);
	cv::imwrite("E:/OneDrive - mail.sdu.edu.cn/202012/DGraph/output/" + std::to_string(index) + ".jpg", img);
}


