
int main(int argc, char* argv[])
{
	int proj_width_ = 1280;
	int proj_height_ = 720;

	std::vector<std::vector<int>> level_images_;

	int tw = 2;
	for (int i = 0; i < 100; i++) {
		tw *= 2;
		if (tw >= proj_width_) break;
	}
	int cur_half = tw / 2;
	std::vector<int> pattern(tw, 1);
	level_images_.push_back(std::vector<int>(tw, 1)); // reference
	level_images_.push_back(std::vector<int>(tw, 0));
	for (int i = 0; i < 50; i++) {
		int j = 0;
		while (true) {
			bool out = false;
			for (int k = 0; k < cur_half; k++) {
				if (j*cur_half + k >= proj_width_) {
					out = true;
					break;
				}
				pattern[j*cur_half + k] = ((j / 2) % 2 == 0) ? (j % 2 ? 0 : 1) : (j % 2 ? 1 : 0);
			}
			if (out) break;
			j++;
		}
		level_images_.push_back(pattern);
		cur_half /= 2;
		if (cur_half == 4) break;
	}



	int level_cnt = level_images_.size();
	LOG("level image count %d\n", level_images_.size());
	for (int j = 0; j < 8; j++) {
		for (int i = 0; i < tw; i++) {
			pattern[i] = ((i + 8 - j) / 4) % 2 ? 0 : 1;
		}
		level_images_.push_back(pattern);
	}
	inverse_1_.resize(tw);
	inverse_2_.resize(tw);
	int j = 0;
	while (true) {
		bool out = false;
		for (int k = 0; k < 8; k++) {
			if (j * 8 + k >= proj_width_) {
				out = true;
				break;
			}
			inverse_1_[j * 8 + k] = (j % 2 == 0) ? 1 : 0;
			inverse_2_[j * 8 + k] = (j % 2 == 0) ? 0 : 1;
		}
		if (out) break;
		j++;
	}
	level_images_.push_back(inverse_1_);
	level_images_.push_back(inverse_2_);

	// RGB인 경우 그리고 프로젝터를 90도 돌린 경우
	unsgned char* proj_buffer_ = (unsigned char*)malloc(proj_width_*proj_height_ * 3 );
	
	for (int i = 0; i < level_images_.size(); i++) {
		for (int x = 0; x < proj_width_; x++) {
			for (int y = 0; y < proj_height_; y++) {
				proj_buffer_[y * proj_width_ * 3 + x * 3 + 0] = level_images_[i][x] * 255;
				proj_buffer_[y * proj_width_ * 3 + x * 3 + 1] = level_images_[i][x] * 255;
				proj_buffer_[y * proj_width_ * 4 + x * 3 + 2] = level_images_[i][x] * 255;
			}
		}
		/// SAVE the Bitmap!!! 
	}
	// projector image generate
}