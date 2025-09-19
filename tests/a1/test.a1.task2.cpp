#include "test.h"
#include "rasterizer/pipeline.h"
#include "rasterizer/programs.h"

#include <limits>
#include <iomanip>
#include <algorithm>
#include <unordered_set>

using TestPipeline = Pipeline<PrimitiveType::Lines, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;

namespace std
{
	template <>
	struct hash<Vec2>
	{
		size_t operator()(const Vec2 &v) const
		{
			static hash<float> hf;
			size_t x = hf(v.x);
			size_t y = hf(v.y);
			return x ^ (y << 16) ^ (y >> (sizeof(y) * 8 - 16));
		}
	};
}

// check that line produces exactly the listed fragments:
void check_line_covers(std::string const &desc, std::vector<Vec2> const &line_strip, std::unordered_set<Vec2> const &expected)
{

	std::unordered_set<Vec2> got;
	for (uint32_t i = 0; i + 1 < line_strip.size(); ++i)
	{
		TestPipeline::ClippedVertex a, b;
		a.fb_position = Vec3(line_strip[i].x, line_strip[i].y, 0.25f);
		a.inv_w = 1.0f;
		a.attributes.fill(1.0f);
		b.fb_position = Vec3(line_strip[i + 1].x, line_strip[i + 1].y, 0.75f);
		b.inv_w = 1.0f;
		b.attributes.fill(2.0f);
		TestPipeline::rasterize_line(a, b, [&](TestPipeline::Fragment const &frag)
									 { got.emplace(frag.fb_position.x, frag.fb_position.y); });
	}

	std::vector<std::string> raster;

	raster.emplace_back(".");

	uint32_t out_of_raster = 0;

	auto draw = [&raster, &out_of_raster](Vec2 const &px, char c)
	{
		int32_t x = int32_t(std::floor(px.x));
		int32_t y = int32_t(std::floor(px.y));

		if (x < 0 || y < 0 || x > 10 || y > 10)
		{
			++out_of_raster;
			return;
		}

		if (uint32_t(y) >= raster.size())
		{
			raster.resize(y + 1, "");
		}
		if (uint32_t(x) >= raster[y].size())
		{
			raster[y].resize(x + 1, '.');
		}
		raster[y][x] = c;
	};

	uint32_t matched = 0;
	uint32_t missed = 0;
	uint32_t extra = 0;

	for (auto const &f : got)
	{
		if ((f.x - std::floor(f.x) != 0.5f) || (f.y - std::floor(f.y) != 0.5f))
		{
			throw Test::error("Rasterizing '" + desc + "', got fragment at (" + std::to_string(f.x) + ", " + std::to_string(f.y) + "), which isn't at a pixel center.");
		}
		if (expected.count(f))
		{
			draw(f, '#');
			++matched;
		}
		else
		{
			draw(f, '!');
			++extra;
		}
	}
	for (auto const &f : expected)
	{
		if (!got.count(f))
		{
			draw(f, '?');
			++missed;
		}
	}

	if (extra > 0 || missed > 0)
	{
		// failed!
		std::string info = "Example '" + desc + "' missed " + std::to_string(missed) + " ('?'); had " + std::to_string(extra) + " extra ('!'); and matched " + std::to_string(matched) + " ('#') fragments:";

		// square up the raster:
		size_t width = 0;
		for (auto const &row : raster)
		{
			width = std::max(width, row.size());
		}
		for (auto &row : raster)
		{
			row.resize(width, '.');
		}

		for (uint32_t y = static_cast<uint32_t>(raster.size()) - 1; y < static_cast<uint32_t>(raster.size()); --y)
		{
			info += "\n    " + raster[y];
		}

		if (out_of_raster)
			info += "\n    (" + std::to_string(out_of_raster) + " out-of-range fragments not plotted.)";

		puts(""); // because "test..."
		info("%s", info.c_str());

		throw Test::error("Example '" + desc + "' didn't match expected.");
	}

	// if nothing extra and nothing missed, success!
	assert(matched == expected.size());
}

// check that line produces exactly the fragments drawn in a fancy picture:
void check_line_covers(std::string const &desc, std::initializer_list<Vec2> const &line_strip, std::initializer_list<std::string> const &raster_)
{
	// convert raster to set of points ( with lower-left being (0,0) ):
	std::vector<std::string> raster(raster_);
	std::unordered_set<Vec2> expected;
	for (uint32_t y = 0; y < raster.size(); ++y)
	{
		std::string const &row = raster[raster.size() - 1 - y];
		for (uint32_t x = 0; x < row.size(); ++x)
		{
			if (row[x] != '.')
			{
				expected.emplace(x + 0.5f, y + 0.5f);
			}
		}
	}
	// use list-of-points version:
	check_line_covers(desc, line_strip, expected);
}

//--------------------------------------------------
// entering/exiting diamond at (1,1):
// only lines that *exit* the diamond should produce a fragment.

Test test_a1_task2_diamond_inside("a1.task2.diamond.inside", []()
								  { check_line_covers(
										"line inside diamond (1,1)",
										{Vec2(1.5f, 1.25f), Vec2(1.25f, 1.5f)},
										{"...",
										 "...",
										 "..."}); });

Test test_a1_task2_diamond_outside("a1.task2.diamond.outside", []()
								   { check_line_covers(
										 "line outside diamond (1,1)",
										 {Vec2(1.125f, 1.25f), Vec2(1.25f, 1.125f)},
										 {"...",
										  "...",
										  "..."}); });

//----------------------------
// simple horizontal and vertical lines (set up so that no enter/exit logic needed):

Test test_a1_task2_simple_horizontal("a1.task2.simple.horizontal", []()
									 { check_line_covers(
										   "horizontal line from (1.125, 1.125) to (4.875, 1.125)",
										   {Vec2(1.125f, 1.125f), Vec2(4.875f, 1.125f)},
										   {"......",
											".####.",
											"......"}); });

Test test_a1_task2_simple_vertical("a1.task2.simple.vertical", []()
								   { check_line_covers(
										 "vertical line from (1.125, 1.125) to (1.125, 4.875)",
										 {Vec2(1.125f, 1.125f), Vec2(1.125f, 4.875f)},
										 {"...",
										  ".#.",
										  ".#.",
										  ".#.",
										  ".#.",
										  "..."}); });

// tests ending point passing through bottom corner
Test test_a1_task2_simple_horizontal1("a1.task2.horizontal.border.bottom.include", []()
									  { check_line_covers(
											"vertical line from (0, 0) to (3.6, 0)",
											{Vec2(0.0f, 0.0f), Vec2(3.6f, 0.0f)},
											{"....",
											 "....",
											 "####"}); });

// tests ending point ending in bottom corner
Test test_a1_task2_simple_horizontal2("a1.task2.horizontal.border.bottom.exclude", []()
									  { check_line_covers(
											"vertical line from (0, 0) to (3.5, 0)",
											{Vec2(0.0f, 0.0f), Vec2(3.5f, 0.0f)},
											{"....",
											 "....",
											 "###."}); });

// tests ending point passing through left corner
Test test_a1_task2_vertical_border2("a1.task2.vertical.border.bottom.left.include", []()
									{ check_line_covers(
										  "vertical line from (1, 0) to (1, 3.6)",
										  {Vec2(1.0f, 0.0f), Vec2(1.0f, 3.6f)},
										  {".#..",
										   ".#..",
										   ".#..",
										   ".#.."}); });

// tests ending point ending in left corner
Test test_a1_task2_vertical_border1("a1.task2.vertical.border.left.exclude", []()
									{ check_line_covers(
										  "line from (1, 0) to (1, 3.5)",
										  {Vec2(1.0f, 0.0f), Vec2(1.0f, 3.5f)},
										  {"....",
										   ".#..",
										   ".#..",
										   ".#.."}); });

// start point should pass diamond exit, end point should not
Test test_a1_task2_diamond_exit("a1.task2.diamond_exit", []()
								{ check_line_covers(
									  "line from (1.5, 1.5) to (2.6, 1.3)",
									  {Vec2(1.5f, 1.5f), Vec2(2.6f, 1.3f)},
									  {".#..",
									   "...."}); });

// start point should pass diamond exit, end point should not
Test test_a1_task2_diamond_exit2("a1.task2.diamond_exit2", []()
								 { check_line_covers(
									   "line from (1.5, 1.5) to (2.5, 1.0)",
									   {Vec2(1.5f, 1.5f), Vec2(2.5f, 1.0f)},
									   {".#..",
										"...."}); });

// start point should pass diamond exit, end point should not
Test test_a1_task2_diamond_exit3("a1.task2.diamond_exit3", []()
								 { check_line_covers(
									   "line from (1.5, 1.5) to (2.0, 1.5)",
									   {Vec2(1.5f, 1.5f), Vec2(2.0f, 1.5f)},
									   {".#..",
										"...."}); });

// both ends should pass diamond exit
Test test_a1_task2_diamond_exit4("a1.task2.diamond_exit4", []()
								 { check_line_covers(
									   "horizontal line from (1.5, 1.5) to (3, 1.5)",
									   {Vec2(1.5f, 1.5f), Vec2(3.0f, 1.5f)},
									   {".##.",
										"...."}); });

// tests if switching point order correctly affects line
Test test_a1_task2_diamond_exit4_swapped("a1.task2.diamond_exit4.swapped", []() { //!!! FIX
	check_line_covers(
		"horizontal line from (3, 1.5) to (1.5, 1.5)",
		{Vec2(3.0f, 1.5f), Vec2(1.5f, 1.5f)},
		{"..##",
		 "...."});
});

// start point should pass diamond exit, end point should not
Test test_a1_task2_diamond_exit5("a1.task2.diamond_exit5", []()
								 { check_line_covers(
									   "horizontal line from (1.5, 1.5) to (2.5, 2.0)",
									   {Vec2(1.5f, 1.5f), Vec2(2.5f, 2.0f)},
									   {".#..",
										"...."}); });

// tests if switching point order correctly affects line
Test test_a1_task2_diamond_exit5_swapped("a1.task2.diamond_exit5.swapped", []() { //!!! FIX
	check_line_covers(
		"horizontal line from (2.5, 2.0) to (1.5, 1.5)",
		{Vec2(2.5f, 2.0f), Vec2(1.5f, 1.5f)},
		{"..#.",
		 "....",
		 "...."});
});

// both points should pass diamond exit
Test test_a1_task2_diamond_exit6("a1.task2.diamond_exit6", []()
								 { check_line_covers(
									   "diagonal line from (1.5, 0.5) to (2.5, 2.0)",
									   {Vec2(1.5f, 0.5f), Vec2(2.5f, 2.0f)},
									   {"..#.",
										".#.."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q3toq2_inclusive("a1.task2.diamond.exit.q3toq2.inclusive", []()
												 { check_line_covers(
													   "vertical line from (1.1, 1.1) to (1.2, 2.9)",
													   {Vec2(1.1f, 1.1f), Vec2(1.2f, 2.9f)},
													   {".#..",
														".#..",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q3toq2_exclusive("a1.task2.diamond.exit.q3toq2.exclusive", []()
												 { check_line_covers(
													   "vertical line from (1.1, 1.1) to (1.2, 2.2)",
													   {Vec2(1.1f, 1.1f), Vec2(1.2f, 2.2f)},
													   {"....",
														".#..",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q3toq4_inclusive("a1.task2.diamond.exit.q3toq4.inclusive", []()
												 { check_line_covers(
													   "horizontal line from (1.1, 1.1) to (2.9, 1.2)",
													   {Vec2(1.1f, 1.1f), Vec2(2.9f, 1.2f)},
													   {"....",
														".##.",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q3toq4_exclusive("a1.task2.diamond.exit.q3toq4.exclusive", []()
												 { check_line_covers(
													   "horizontal line from (1.1, 1.1) to (2.9, 1.2)",
													   {Vec2(1.1f, 1.1f), Vec2(2.4f, 1.2f)},
													   {"....",
														".#..",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q1toq2_inclusive("a1.task2.diamond.exit.q1toq2.inclusive", []()
												 { check_line_covers(
													   "horizontal line from (2.9, 1.9) to (1.2, 1.8)",
													   {Vec2(2.9f, 1.9f), Vec2(1.2f, 1.8f)},
													   {"....",
														".##.",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q1toq2_exclusive("a1.task2.diamond.exit.q1toq2.exclusive", []()
												 { check_line_covers(
													   "horizontal line from (2.1, 1.9) to (1.2, 1.8)",
													   {Vec2(2.1f, 1.9f), Vec2(1.2f, 1.8f)},
													   {"....",
														".#..",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q1toq4_inclusive("a1.task2.diamond.exit.q1toq4.inclusive", []()
												 { check_line_covers(
													   "vertical line from (1.7, 2.8) to (1.9, 1.1)",
													   {Vec2(1.7f, 2.8f), Vec2(1.9f, 1.1f)},
													   {".#..",
														".#..",
														"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q1toq4_exclusive("a1.task2.diamond.exit.q1toq4.exclusive", []()
												 { check_line_covers(
													   "vertical line from (1.9, 1.1) to (1.7, 2.6)",
													   {Vec2(1.9f, 1.1f), Vec2(1.7f, 2.6f)},
													   {"....",
														".#..",
														"...."}); });

// tests if switching point order correctly affects line
Test test_a1_task2_diamond_exit_q1toq4_exclusive_swapped("a1.task2.diamond.exit.q1toq4.exclusive.swapped", []()
														 { check_line_covers(
															   "vertical line from (1.7, 2.6) to (1.9, 1.1)",
															   {Vec2(1.7f, 2.6f), Vec2(1.9f, 1.1f)},
															   {".#..",
																".#..",
																"...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q1toq3("a1.task2.diamond.exit.q1toq3", []()
									   { check_line_covers(
											 "vertical line from (1.1, 2.9) to (1.9, 2.1)",
											 {Vec2(1.1f, 2.9f), Vec2(1.9f, 2.1f)},
											 {".#..",
											  "....",
											  "...."}); });

// testing crossing the diamond from different quadrants
Test test_a1_task2_diamond_exit_q2toq4("a1.task2.diamond.exit.q2toq4", []()
									   { check_line_covers(
											 "vertical line from (1.7, 2.8) to (1.1, 2.1)",
											 {Vec2(1.7f, 2.8f), Vec2(1.1f, 2.1f)},
											 {".#..",
											  "....",
											  "...."}); });

// testing 1px diamond exit cases
Test test_a1_task2_diamond_exit_1px_1("a1.task2.diamond.exit.1px.1", []()
									  { check_line_covers(
											"vertical line from (1.1, 1.1) to (1.2, 1.9)",
											{Vec2(1.1f, 1.1f), Vec2(1.2f, 1.9f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing 1px diamond exit cases
Test test_a1_task2_diamond_exit_1px_2("a1.task2.diamond.exit.1px.2", []()
									  { check_line_covers(
											"vertical line from (1.5, 1.5) to (1.1, 1.9)",
											{Vec2(1.5f, 1.5f), Vec2(1.2f, 1.9f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing 1px diamond exit cases
Test test_a1_task2_diamond_exit_1px_3("a1.task2.diamond.exit.1px.3", []()
									  { check_line_covers(
											"vertical line from (1.5, 1.5) to (1.9, 1.9)",
											{Vec2(1.5f, 1.5f), Vec2(1.9f, 1.9f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing 1px diamond exit cases
Test test_a1_task2_diamond_exit_1px_4("a1.task2.diamond.exit.1px.4", []()
									  { check_line_covers(
											"vertical line from (1.5, 1.5) to (1.9, 1.2)",
											{Vec2(1.5f, 1.5f), Vec2(1.9f, 1.2f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing 1px diamond exit cases
Test test_a1_task2_diamond_exit_1px_5("a1.task2.diamond.exit.1px.5", []()
									  { check_line_covers(
											"vertical line from (1.5, 1.5) to (1.2, 1.2)",
											{Vec2(1.5f, 1.5f), Vec2(1.2f, 1.2f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_diamond_cross_left("a1.task2.diamond.cross.left", []()
									  { check_line_covers(
											"vertical line from (1, 1) to (1, 2)",
											{Vec2(1.0f, 1.0f), Vec2(1.0f, 2.0f)},
											{"....",
											 ".#..",
											 "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_diamond_cross_bottom("a1.task2.diamond.cross.bottom", []()
										{ check_line_covers(
											  "vertical line from (1, 1) to (2, 1)",
											  {Vec2(1.0f, 1.0f), Vec2(2.0f, 1.0f)},
											  {"....",
											   ".#..",
											   "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_diamond_cross_right("a1.task2.diamond.cross.right", []()
									   { check_line_covers(
											 "vertical line from (2, 2) to (2, 1)",
											 {Vec2(2.0f, 2.0f), Vec2(2.0f, 1.0f)},
											 {"....",
											  "..#.",
											  "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_diamond_cross_top("a1.task2.diamond.cross.top", []()
									 { check_line_covers(
										   "vertical line from (2, 2) to (1, 2)",
										   {Vec2(2.0f, 2.0f), Vec2(1.0f, 2.0f)},
										   {".#..",
											"....",
											"...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_never_leaves_q1("a1.task2.never.leaves.q1", []()
								   { check_line_covers(
										 "vertical line from (11.9, 1.9) to (1.8, 1.6)",
										 {Vec2(1.9f, 1.9f), Vec2(1.8f, 1.6f)},
										 {"....",
										  "....",
										  "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_never_leaves_q2("a1.task2.never.leaves.q2", []()
								   { check_line_covers(
										 "vertical line from (1, 1.6) to (1.2, 1.9)",
										 {Vec2(1.0f, 1.6f), Vec2(1.2f, 1.9f)},
										 {"....",
										  "....",
										  "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_never_leaves_q3("a1.task2.never.leaves.q3", []()
								   { check_line_covers(
										 "vertical line from (1, 1) to (1, 1.3)",
										 {Vec2(1.0f, 1.0f), Vec2(1.2f, 1.3f)},
										 {"....",
										  "....",
										  "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_never_leaves_q4("a1.task2.never.leaves.q4", []()
								   { check_line_covers(
										 "vertical line from (1.9, 1) to (1.6, 1.3)",
										 {Vec2(1.9f, 1.0f), Vec2(1.6f, 1.3f)},
										 {"....",
										  "....",
										  "...."}); });

// testing crossing the diamonds at each diamond vertex
Test test_a1_task2_never_leaves_diamond("a1.task2.never.leaves.diamond", []()
										{ check_line_covers(
											  "vertical line from (1.5, 1.5) to (1.1, 1.5)",
											  {Vec2(1.5f, 1.5f), Vec2(1.1f, 1.5f)},
											  {"....",
											   "....",
											   "...."}); });

// never leaves the diamond edge cases
Test test_a1_task2_never_leaves_diamond1("a1.task2.never.leaves.diamond1", []()
										 { check_line_covers(
											   "vertical line from (1.5, 1.5) to (1.1, 1.5)",
											   {Vec2(1.0f, 1.5f), Vec2(1.5f, 1.0f)},
											   {"....",
												"....",
												"...."}); });

// never leaves the diamond edge cases
Test test_a1_task2_never_leaves_diamond2("a1.task2.never.leaves.diamond2", []()
										 { check_line_covers(
											   "vertical line from (1.07, 1.43) to (1.22, 1.28)",
											   {Vec2(1.07f, 1.43f), Vec2(1.22f, 1.28f)},
											   {"....",
												"....",
												"...."}); });

// never leaves the diamond edge cases
Test test_a1_task2_never_leaves_diamond3("a1.task2.never.leaves.diamond3", []()
										 { check_line_covers(
											   "vertical line from (1.31f, 1.81f) to (1.14f, 1.64f)",
											   {Vec2(1.31f, 1.81f), Vec2(1.14f, 1.64f)},
											   {"....",
												"....",
												"...."}); });

// never leaves the diamond edge cases
Test test_a1_task2_never_leaves_diamond4("a1.task2.never.leaves.diamond4", []()
										 { check_line_covers(
											   "vertical line from (1.64f, 1.86f) to (1.86f, 1.64f)",
											   {Vec2(1.64f, 1.86f), Vec2(1.86f, 1.64f)},
											   {"....",
												"....",
												"...."}); });

// never leaves the diamond edge cases
Test test_a1_task2_never_leaves_diamond5("a1.task2.never.leaves.diamond5", []()
										 { check_line_covers(
											   "vertical line from (1.88f, 1.38f) to (1.38f, 1.88f)",
											   {Vec2(1.88f, 1.38f), Vec2(1.38f, 1.88f)},
											   {"....",
												"....",
												"...."}); });

// never enters the diamond cases
Test test_a1_task2_never_enters_diamond_horizontal1("a1.task2.never.enters.diamond.horizontal1", []()
													{ check_line_covers(
														  "vertical line from (0.9f, 1.9f) to (1.2f, 1.8f)",
														  {Vec2(0.9f, 1.9f), Vec2(1.2f, 1.8f)},
														  {"....",
														   "....",
														   "...."}); });

// never enters the diamond cases
Test test_a1_task2_never_enters_diamond_horizontal2("a1.task2.never.enters.diamond.horizontal2", []()
													{ check_line_covers(
														  "vertical line from (0.9f, 1.2f) to (1.3f, 1.3f)",
														  {Vec2(0.9f, 1.2f), Vec2(1.3f, 1.3f)},
														  {"....",
														   "....",
														   "...."}); });

// never enters the diamond cases
Test test_a1_task2_never_enters_diamond_vertical1("a1.task2.never.enters.diamond.vertical1", []()
												  { check_line_covers(
														"vertical line from (1.1f, 1.9f) to (1.2f, 2.1f)",
														{Vec2(1.1f, 1.9f), Vec2(1.2f, 2.1f)},
														{"....",
														 "....",
														 "...."}); });

// never enters the diamond cases
Test test_a1_task2_never_enters_diamond_vertical2("a1.task2.never.enters.diamond.vertical2", []()
												  { check_line_covers(
														"vertical line from (1.6f, 1.9f) to (1.8f, 2.1f)",
														{Vec2(1.7f, 1.9f), Vec2(1.8f, 2.1f)},
														{"....",
														 "....",
														 "...."}); });

// doesn't exits diamond edge case
Test test_a1_task2_no_exits_diamond_edge1("a1.task2.no.exits.diamond.edge1", []()
										  { check_line_covers(
												"vertical line from (1.5f, 1.5f) to (1.5f, 1.0f)",
												{Vec2(1.5f, 1.5f), Vec2(1.5f, 1.0f)},
												{"....",
												 "....",
												 "...."}); });

// doesn't exits diamond eddge case
Test test_a1_task2_no_exits_diamond_edge2("a1.task2.no.exits.diamond.edge2", []()
										  { check_line_covers(
												"vertical line from (1.5f, 1.5f) to (1.5f, 1.0f)",
												{Vec2(1.5f, 1.5f), Vec2(1.0f, 1.5f)},
												{"....",
												 "....",
												 "...."}); });

// doesn't exits diamond eddge case
Test test_a1_task2_no_exits_diamond_edge3("a1.task2.no.exits.diamond.edge3", []()
										  { check_line_covers(
												"vertical line from (0.5f, 0.5f) to (1.0f, 1.5f)",
												{Vec2(0.5f, 0.5f), Vec2(1.0f, 1.5f)},
												{"....",
												 "....",
												 "#..."}); });

// doesn't exits diamond eddge case
Test test_a1_task2_no_exits_diamond_edge4("a1.task2.no.exits.diamond.edge4", []()
										  { check_line_covers(
												"vertical line from (0.5f, 0.5f) to (1.5f, 1.0f)",
												{Vec2(0.5f, 0.5f), Vec2(1.5f, 1.0f)},
												{"....",
												 "....",
												 "#..."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge1("a1.task2.exits.diamond.edge1", []()
									   { check_line_covers(
											 "vertical line from (1.5f, 1.5f) to (1.5f, 1.0f)",
											 {Vec2(1.5f, 1.5f), Vec2(1.5f, 2.0f)},
											 {"....",
											  ".#..",
											  "...."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge2("a1.task2.exits.diamond.edge2", []()
									   { check_line_covers(
											 "vertical line from (1.5f, 1.5f) to (1.5f, 1.0f)",
											 {Vec2(1.5f, 1.5f), Vec2(2.0f, 1.5f)},
											 {"....",
											  ".#..",
											  "...."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge3("a1.task2.exits.diamond.edge3", []()
									   { check_line_covers(
											 "vertical line from (1.0f, 1.5f) to (0.5f, 0.5f)",
											 {Vec2(1.0f, 1.5f), Vec2(0.5f, 0.5f)},
											 {"....",
											  ".#..",
											  "...."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge4("a1.task2.exits.diamond.edge4", []()
									   { check_line_covers(
											 "vertical line from (1.0f, 1.5f) to (0.5f, 0.5f)",
											 {Vec2(1.5f, 1.0f), Vec2(0.5f, 0.5f)},
											 {"....",
											  ".#..",
											  "...."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge5("a1.task2.exits.diamond.edge5", []()
									   { check_line_covers(
											 "vertical line from 0.5f, 0.5f) to (1.5f, 2.0f)",
											 {Vec2(0.5f, 0.5f), Vec2(1.5f, 2.0f)},
											 {"....",
											  ".#..",
											  "#..."}); });

// exits diamond edge case
Test test_a1_task2_exits_diamond_edge6("a1.task2.exits.diamond.edge6", []()
									   { check_line_covers(
											 "vertical line from 0.5f, 0.5f) to (2.0f, 1.5f)",
											 {Vec2(0.5f, 0.5f), Vec2(2.0f, 1.5f)},
											 {"....",
											  ".#..",
											  "#..."}); });

// inside diamond edge cases
Test test_a1_task2_inside_diamond1("a1.task2.inside.diamond1", []()
								   { check_line_covers(
										 "vertical line from (1.5f, 1.0f) to (1.5f, 2.0f)",
										 {Vec2(1.5f, 1.0f), Vec2(1.5f, 2.0f)},
										 {"....",
										  ".#..",
										  "...."}); });

// inside diamond edge cases
Test test_a1_task2_inside_diamond1_swapped("a1.task2.inside.diamond1.swapped", []()
										   { check_line_covers(
												 "vertical line from (1.5f, 2.0f) to (1.5f, 1.0f)",
												 {Vec2(1.5f, 2.0f), Vec2(1.5f, 1.0f)},
												 {".#..",
												  "....",
												  "...."}); });

// inside diamond edge cases
Test test_a1_task2_inside_diamond2("a1.task2.inside.diamond2", []()
								   { check_line_covers(
										 "vertical line from (1.0f, 1.5f) to (2.0f, 1.5f)",
										 {Vec2(1.0f, 1.5f), Vec2(2.0f, 1.5f)},
										 {"....",
										  ".#..",
										  "...."}); });

// inside diamond edge cases
Test test_a1_task2_inside_diamond2_swapped("a1.task2.inside.diamond2.swapped", []()
										   { check_line_covers(
												 "vertical line from (1.5f, 2.0f) to (1.5f, 1.0f) swapped",
												 {Vec2(2.0f, 1.5f), Vec2(1.0f, 1.5f)},
												 {"....",
												  "..#.",
												  "...."}); });

// 45 degree non edge case
Test test_a1_task2_45deg("a1.task2.inside.45deg", []()
						 { check_line_covers(
							   "vertical line from Vec2(0.5f, 0.5f), Vec2(3.5f, 3.5f)",
							   {Vec2(0.5f, 0.5f), Vec2(3.5f, 3.5f)},
							   {"..#.",
								".#..",
								"#..."}); });

// lines
Test test_a1_task2_line_horizontal("a1.task2.inside.line.horizontal", []()
								   { check_line_covers(
										 "vertical line from Vec2(0.5f, 0.5f), Vec2(4.4f, 2.4f)",
										 {Vec2(0.5f, 0.5f), Vec2(4.4f, 2.4f)},
										 {"....",
										  "..##",
										  "##.."}); });

// lines
Test test_a1_task2_line_vertical("a1.task2.inside.line.vertical", []()
								 { check_line_covers(
									   "vertical line from Vec2(0.5f, 0.5f), Vec2(1.3f, 4.3f)",
									   {Vec2(0.5f, 0.5f), Vec2(1.3f, 4.3f)},
									   {".#..",
										"#...",
										"#...",
										"#..."}); });

Test test_a1_task2_45deg_left_to_top("a1.task2.45deg.left2top", []()
									 { check_line_covers("45 degree line from (0, 0.5) to (0.5, 1)",

														 {Vec2(0.0f, 0.5f), Vec2(0.5f, 1.0f)},

														 {"...",

														  "#.."}

									   ); });

Test test_a1_task2_45deg_top_to_left("a1.task2.45deg.top2left", []()
									 { check_line_covers("45 degree line from (0.5, 1) to (0, 0.5)",

														 {Vec2(0.5f, 1.0f), Vec2(0.0f, 0.5f)},

														 {"#..",

														  "..."}

									   ); });

Test test_a1_task2_45deg_left_to_bottom("a1.task2.45deg.left2bottom", []()
										{ check_line_covers("45 degree line from (0, 0.5) to (0.5, 0)",

															{Vec2(0.0f, 0.5f), Vec2(0.5f, 0.0f)},

															{"...",

															 "..."}

										  ); });

Test test_a1_task2_45deg_bottom_to_left("a1.task2.45deg.bottom2left.", []()
										{ check_line_covers("45 degree line from (0, 0.5) to (0.5, 0)",

															{Vec2(0.5f, 0.0f), Vec2(0.0f, 0.5f)},

															{"...",

															 "..."}

										  ); });

Test test_a1_task2_45deg_bottom_to_right("a1.task2.45deg.bottom2right", []()
										 { check_line_covers("45 degree line from (0.5, 0) to (1, 0.5)",

															 {Vec2(0.5f, 0.0f), Vec2(1.0f, 0.5f)},

															 {"...",

															  "#.."}

										   ); });

Test test_a1_task2_45deg_right_to_bottom("a1.task2.45deg.right2bottom", []()
										 { check_line_covers("45 degree line from (1, 0.5) to (0.5, 0)",

															 {Vec2(1.0f, 0.5f), Vec2(0.5f, 0.0f)},

															 {"...",

															  ".#."}

										   ); });

Test test_a1_task2_45deg_top_to_right("a1.task2.45deg.top2right", []()
									  { check_line_covers("45 degree line from (0.5, 1) to (1, 0.5)",

														  {Vec2(0.5f, 1.0f), Vec2(1.0f, 0.5f)},

														  {"#..",

														   "..."}

										); });

Test test_a1_task2_45deg_right_to_top("a1.task2.45deg.right2top", []()
									  { check_line_covers("45 degree line from (1, 0.5) to (0.5, 1)",

														  {Vec2(1.0f, 0.5f), Vec2(0.5f, 1.0f)},

														  {"...",

														   ".#."}

										); });

Test test_a1_task2_45deg_aaa("a1.task2.45deg.aaa", []()
							 { check_line_covers("45 degree line from (0.5, 0) to (1.5, 1)",

												 {Vec2(0.5f, 0.0f), Vec2(1.5f, 1.0f)},

												 {"...",

												  "...",

												  "##."}

							   ); });

Test test_a1_task2_45deg_bbb("a1.task2.45deg.bbb", []()
							 { check_line_covers("45 degree line from (0, 0.5) to (1, 1.5)",

												 {Vec2(0.0f, 0.5f), Vec2(1.0f, 1.5f)},

												 {"...",

												  "#..",

												  "#.."}

							   ); });

Test test_a1_task2_45deg_ccc("a1.task2.45deg.ccc", []()
							 { check_line_covers("45 degree line from (0.5, 0) to (1.5, 1)",

												 {Vec2(1.5f, 1.0f), Vec2(0.5f, 0.0f)},

												 {"...",

												  ".#.",

												  ".#."}

							   ); });

Test test_a1_task2_45deg_ddd("a1.task2.45deg.ddd", []()
							 { check_line_covers("45 degree line from (0, 0.5) to (1, 1.5)",

												 {Vec2(1.0f, 1.5f), Vec2(0.0f, 0.5f)},

												 {"...",

												  "##.",

												  "..."}

							   ); });