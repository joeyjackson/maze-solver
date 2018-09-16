#include "Main.h"

int main(int argc, char** argv) {
	std::string file_path;
	char filename[MAX_PATH];

	OPENFILENAME ofn;
	ZeroMemory(&filename, sizeof(filename));
	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = NULL;
	ofn.lpstrFilter = "PNG Files\0*.png";
	ofn.lpstrFile = filename;
	ofn.nMaxFile = MAX_PATH;
	ofn.lpstrTitle = "Select a maze image file";
	ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST;

	if (GetOpenFileNameA(&ofn)) {
		file_path = std::string(filename);
	} else {
		switch (CommDlgExtendedError())
		{
		case CDERR_DIALOGFAILURE: std::cout << "CDERR_DIALOGFAILURE\n";   break;
		case CDERR_FINDRESFAILURE: std::cout << "CDERR_FINDRESFAILURE\n";  break;
		case CDERR_INITIALIZATION: std::cout << "CDERR_INITIALIZATION\n";  break;
		case CDERR_LOADRESFAILURE: std::cout << "CDERR_LOADRESFAILURE\n";  break;
		case CDERR_LOADSTRFAILURE: std::cout << "CDERR_LOADSTRFAILURE\n";  break;
		case CDERR_LOCKRESFAILURE: std::cout << "CDERR_LOCKRESFAILURE\n";  break;
		case CDERR_MEMALLOCFAILURE: std::cout << "CDERR_MEMALLOCFAILURE\n"; break;
		case CDERR_MEMLOCKFAILURE: std::cout << "CDERR_MEMLOCKFAILURE\n";  break;
		case CDERR_NOHINSTANCE: std::cout << "CDERR_NOHINSTANCE\n";     break;
		case CDERR_NOHOOK: std::cout << "CDERR_NOHOOK\n";          break;
		case CDERR_NOTEMPLATE: std::cout << "CDERR_NOTEMPLATE\n";      break;
		case CDERR_STRUCTSIZE: std::cout << "CDERR_STRUCTSIZE\n";      break;
		case FNERR_BUFFERTOOSMALL: std::cout << "FNERR_BUFFERTOOSMALL\n";  break;
		case FNERR_INVALIDFILENAME: std::cout << "FNERR_INVALIDFILENAME\n"; break;
		case FNERR_SUBCLASSFAILURE: std::cout << "FNERR_SUBCLASSFAILURE\n"; break;
		default: std::cout << "You cancelled.\n";
		}
	}

	//std::string path = "C:\\Users\\ejjac\\Documents\\CppProjs\\OpenCV_Proj\\maze-solver\\";
	//std::string fname = "maze4";

	cv::Mat image = cv::imread(file_path);

	MazeSolver solver(image);
	//PrintGraph(*(solver.getGraph()));
	fprintf(stderr, "Graph Created\n");
	std::tuple<float, std::vector<Edge<Point>>> solution = solver.solve();
	fprintf(stderr, "Maze Solved\n");
	std::vector<Edge<Point>> solution_edges = std::get<1>(solution);

	cv::Mat solution_img = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

	for (Edge<Point> edge : *(solver.getGraph()->getEdges())) {
		cv::Point p_u(edge.getU()->getData()->x, edge.getU()->getData()->y);
		cv::Point p_v(edge.getV()->getData()->x, edge.getV()->getData()->y);
		cv::line(solution_img, p_u, p_v, cv::Scalar(255, 255, 255), 1);
	}

	for (Edge<Point> edge : solution_edges) {
		cv::Point p_u(edge.getU()->getData()->x, edge.getU()->getData()->y);
		cv::Point p_v(edge.getV()->getData()->x, edge.getV()->getData()->y);
		cv::line(solution_img, p_u, p_v, cv::Scalar(255, 0, 0), 1);
	}

	if (solution_img.rows < 300) {
		image_resize(solution_img, solution_img, 200, -1, cv::INTER_NEAREST);
	} 
	if (solution_img.cols < 300) {
		image_resize(solution_img, solution_img, -1, 200, cv::INTER_NEAREST);
	}

	cv::imwrite(file_path.substr(0, file_path.size() - 4) + "_solution.png", solution_img);

	const char * window_name = "MAZE SOLUTION";
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	while (cvGetWindowHandle(window_name)) {
		cv::imshow(window_name, solution_img);
		cv::waitKey(1);
	}
	cv::destroyAllWindows();
	return 0;
}

void image_resize(const cv::Mat& img, cv::Mat& dst, int height, int width, int inter) {
	if (height < 0 && width < 0) {
		height = img.rows;
		width = img.cols;
	}
	else if (height < 0) {
		double ratio = (double)width / (double)img.cols;
		height = (int)(img.rows * ratio);
	}
	else if (width < 0) {
		double ratio = (double)height / (double)img.rows;
		width = (int)(img.cols * ratio);
	}

	cv::resize(img, dst, cv::Size(width, height), 0, 0, inter);
}
