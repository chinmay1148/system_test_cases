#include <gtest/gtest.h>

#include "kimera-vio/dataprovider/EurocDataProvider.h"

class DatasetParserTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Initialize with some default parameters for VioParams
    // This is a mock, you would need to ensure that VioParams can be 
    // initialized in this way or use a suitable method.
    vio_params_ = std::make_shared<VIO::VioParams>("<Path to your params>");
  }

  VIO::VioParams::Ptr vio_params_;
};

TEST_F(DatasetParserTest, EurocDatasetParserInitialization) {
  VIO::DataProviderInterface::Ptr dataset_parser;

  int dataset_type = 0; // For Euroc
  switch (dataset_type) {
    case 0: {
      dataset_parser = VIO::make_unique<VIO::MonoEurocDataProvider>(*vio_params_);
    } break;
    // ... (rest of the cases)
  }

  ASSERT_NE(dataset_parser, nullptr);
  ASSERT_TRUE(std::dynamic_pointer_cast<VIO::MonoEurocDataProvider>(dataset_parser) != nullptr);
}

// Add more TEST_F for other functionalities you want to test.

TEST_F(DatasetParserTest, MonoImuPipelineInitialization) {
  VIO::Pipeline::Ptr vio_pipeline;

  vio_params_->frontend_type_ = VIO::FrontendType::kMonoImu;
  switch (vio_params_->frontend_type_) {
    case VIO::FrontendType::kMonoImu: {
      vio_pipeline = VIO::make_unique<VIO::MonoImuPipeline>(*vio_params_);
    } break;
    // ... (other cases if added)
  }

  ASSERT_NE(vio_pipeline, nullptr);
  ASSERT_TRUE(std::dynamic_pointer_cast<VIO::MonoImuPipeline>(vio_pipeline) != nullptr);
}

TEST_F(DatasetParserTest, StereoImuPipelineInitialization) {
  VIO::Pipeline::Ptr vio_pipeline;

  vio_params_->frontend_type_ = VIO::FrontendType::kStereoImu;
  switch (vio_params_->frontend_type_) {
    case VIO::FrontendType::kStereoImu: {
      vio_pipeline = VIO::make_unique<VIO::StereoImuPipeline>(*vio_params_);
    } break;
    // ... (other cases if added)
  }

  ASSERT_NE(vio_pipeline, nullptr);
  ASSERT_TRUE(std::dynamic_pointer_cast<VIO::StereoImuPipeline>(vio_pipeline) != nullptr);
}

TEST_F(DatasetParserTest, RegisterShutdownCallback) {
  VIO::Pipeline::Ptr vio_pipeline = VIO::make_unique<VIO::MonoImuPipeline>(*vio_params_);
  VIO::DataProviderInterface::Ptr dataset_parser = VIO::make_unique<VIO::MonoEurocDataProvider>(*vio_params_);

  bool callback_registered = false;
  try {
    vio_pipeline->registerShutdownCallback(
      std::bind(&VIO::DataProviderInterface::shutdown, dataset_parser));
    callback_registered = true;
  } catch (...) {
    callback_registered = false;
  }

  ASSERT_TRUE(callback_registered);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
