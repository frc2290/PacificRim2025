package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.mockito.Mockito.lenient;
import static org.mockito.Mockito.when;

import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

@ExtendWith(MockitoExtension.class)
class VisionUtilsTest {
  @Mock private PhotonCamera camera;
  private VisionUtils vision;

  @BeforeEach
  void setUp() {
    lenient().when(camera.isConnected()).thenReturn(true);
    vision = new VisionUtils(camera);
  }

  @Test
  void getLatestResultReturnsEmptyWhenNoResults() {
    when(camera.getAllUnreadResults()).thenReturn(List.of());
    PhotonPipelineResult result = vision.getLatestResult();
    assertEquals(0, result.getTargets().size());
  }

  @Test
  void getLatestResultReturnsMostRecent() {
    PhotonPipelineResult first = new PhotonPipelineResult();
    PhotonPipelineResult second = new PhotonPipelineResult();
    when(camera.getAllUnreadResults()).thenReturn(List.of(first, second));
    assertSame(second, vision.getLatestResult());
  }

  @Test
  void periodicUpdatesDetections() {
    PhotonPipelineResult result = new PhotonPipelineResult();
    when(camera.getAllUnreadResults()).thenReturn(List.of(result));
    vision.periodic();
    assertSame(result, vision.getDetections());
  }

  @Test
  void periodicHandlesDisconnectedCamera() {
    when(camera.isConnected()).thenReturn(false);
    vision.periodic();
    assertEquals(0, vision.getDetections().getTargets().size());
    assertFalse(vision.isConnected());
  }
}
