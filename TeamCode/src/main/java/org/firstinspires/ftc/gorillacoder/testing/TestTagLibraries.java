package org.firstinspires.ftc.gorillacoder.testing;

import java.util.ArrayList;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMemberMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class TestTagLibraries {

    public static final double tag250SizeInches = 2.5;

    public static final double tag325SizeInches = 3.25;

    public static final double tag350SizeInches = 3.5;

    public static final double tag400SizeInches = 4.0;

    Quaternion facingMinusY = new Quaternion(0.0f,    0.0f, 0.0f,     1.0f, 0L); // 180 deg (Front Wall)
    Quaternion facingPlusY  = new Quaternion(1.0f,    0.0f, 0.0f,     0.0f, 0L); // 0 deg   (Back Wall)
    Quaternion facingPlusX  = new Quaternion(0.7071f, 0.0f, 0.0f,  0.7071f, 0L); // +90 deg (Red Wall)
    Quaternion facingMinusX = new Quaternion(0.7071f, 0.0f, 0.0f, -0.7071f, 0L); // -90 deg (Blue Wall)

    public AprilTagLibrary createTagLibrary_WallSingleCentered() {
        AprilTagLibrary result = new AprilTagLibrary.Builder()
            // Tag 1: Front Wall (+Y) - Facing inside toward -Y
            .addTag(1, "Front Wall", tag400SizeInches,
                new VectorF(0.0f, 72.0f, 12.0f), DistanceUnit.INCH, facingMinusY)

            // Tag 2: Back Wall (-Y) - Facing inside toward +Y
            .addTag(2, "Back Wall", tag400SizeInches,
                new VectorF(0.0f, -72.0f, 12.0f), DistanceUnit.INCH, facingPlusY)

            // Tag 3: Red Wall (-X) - Facing inside toward +X
            .addTag(3, "Red Wall", tag400SizeInches,
                new VectorF(-72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingPlusX)

            // Tag 4: Blue Wall (+X) - Facing inside toward -X
            .addTag(4, "Blue Wall", tag400SizeInches,
                new VectorF(72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingMinusX)
            .build();

        return result;
    }

    // Use BioBuzz clusters
    public AprilTagLibrary createTagLibrary_WallBioBuzz2LeftClusterCentered() {
        // Define Members

        // Front Wall Cluster Members
        // BioBuzz RSLeft
        AprilTagClusterMemberMetadata frontMemberA = new AprilTagClusterMemberMetadata(30, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata frontMemberB = new AprilTagClusterMemberMetadata(31, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);

        // Back Wall Cluster Members
        // BioBuzz BSLeft
        AprilTagClusterMemberMetadata backMemberA = new AprilTagClusterMemberMetadata(42, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backMemberB = new AprilTagClusterMemberMetadata(43, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);

        // Red Wall Cluster Members
        // BioBuzz RALeft
        AprilTagClusterMemberMetadata redMemberA = new AprilTagClusterMemberMetadata(34, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redMemberB = new AprilTagClusterMemberMetadata(35, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);

        // Blue Wall Cluster Members
        // BioBuzz BALeft
        AprilTagClusterMemberMetadata blueMemberA = new AprilTagClusterMemberMetadata(38, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueMemberB = new AprilTagClusterMemberMetadata(39, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);

        // Define Clusters

        // Front Wall Cluster
        AprilTagClusterMetadata frontCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(frontMemberA, frontMemberB)),
            "Front", "Front", new VectorF(0.0f, 72.0f, 12.0f), DistanceUnit.INCH, facingMinusY
        );

        // Back Wall Cluster
        AprilTagClusterMetadata backCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(backMemberA, backMemberB)),
            "Back", "Back", new VectorF(0.0f, -72.0f, 12.0f), DistanceUnit.INCH, facingPlusY
        );

        // Red Wall Cluster
        AprilTagClusterMetadata redCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(redMemberA, redMemberB)),
            "Red", "Red", new VectorF(-72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingPlusX
        );

        // Blue Wall Wall Cluster
        AprilTagClusterMetadata blueCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(blueMemberA, blueMemberB)),
            "Blue", "Blue", new VectorF(72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingMinusX
        );

// 4. Add clusters to the library mapping builder
        AprilTagLibrary result = new AprilTagLibrary.Builder()
            .addCluster(frontCluster)
            .addCluster(backCluster)
            .addCluster(redCluster)
            .addCluster(blueCluster)
            .build();

        return result;
    }

    public AprilTagLibrary createTagLibrary_WallBioBuzz4ClusterCentered() {
// 2. Define 4-Tag Cluster Members (IDs 30 through 45)
// Calculated relative positions from array center: -1.625, -1.125, +1.125, +1.625

// front Wall Cluster (Offsets along local X axis)
        AprilTagClusterMemberMetadata audMember1 = new AprilTagClusterMemberMetadata(30, new VectorF(-1.625f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audMember2 = new AprilTagClusterMemberMetadata(31, new VectorF(-1.125f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audMember3 = new AprilTagClusterMemberMetadata(32, new VectorF( 1.125f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audMember4 = new AprilTagClusterMemberMetadata(33, new VectorF( 1.625f, 0.0f, 0.0f), tag325SizeInches);

// Back Wall Cluster (Offsets along local X axis)
        AprilTagClusterMemberMetadata backMember1 = new AprilTagClusterMemberMetadata(34, new VectorF(-1.625f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backMember2 = new AprilTagClusterMemberMetadata(35, new VectorF(-1.125f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backMember3 = new AprilTagClusterMemberMetadata(36, new VectorF( 1.125f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backMember4 = new AprilTagClusterMemberMetadata(37, new VectorF( 1.625f, 0.0f, 0.0f), tag325SizeInches);

// Red Wall Cluster (Offsets along local Y axis)
        AprilTagClusterMemberMetadata redMember1 = new AprilTagClusterMemberMetadata(38, new VectorF(0.0f, -1.625f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redMember2 = new AprilTagClusterMemberMetadata(39, new VectorF(0.0f, -1.125f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redMember3 = new AprilTagClusterMemberMetadata(40, new VectorF(0.0f,  1.125f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redMember4 = new AprilTagClusterMemberMetadata(41, new VectorF(0.0f,  1.625f, 0.0f), tag325SizeInches);

// Blue Wall Cluster (Offsets along local Y axis)
        AprilTagClusterMemberMetadata blueMember1 = new AprilTagClusterMemberMetadata(42, new VectorF(0.0f, -1.625f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueMember2 = new AprilTagClusterMemberMetadata(43, new VectorF(0.0f, -1.125f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueMember3 = new AprilTagClusterMemberMetadata(44, new VectorF(0.0f,  1.125f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueMember4 = new AprilTagClusterMemberMetadata(45, new VectorF(0.0f,  1.625f, 0.0f), tag325SizeInches);

// 3. Instantiate AprilTagClusterMetadata groups
// Parameters: members, name, shortName, fieldPosition, distanceUnit, fieldOrientation
        AprilTagClusterMetadata frontCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(audMember1, audMember2, audMember3, audMember4)),
            "Front", "Front", new VectorF(0.0f, 72.0f, 12.0f), DistanceUnit.INCH, facingMinusY
        );

        AprilTagClusterMetadata backCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(backMember1, backMember2, backMember3, backMember4)),
            "Back", "Back", new VectorF(0.0f, -72.0f, 12.0f), DistanceUnit.INCH, facingPlusY
        );

        AprilTagClusterMetadata redCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(redMember1, redMember2, redMember3, redMember4)),
            "Red", "Red", new VectorF(-72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingPlusX
        );

        AprilTagClusterMetadata blueCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(blueMember1, blueMember2, blueMember3, blueMember4)),
            "Blue", "Blue", new VectorF(72.0f, 0.0f, 12.0f), DistanceUnit.INCH, facingMinusX
        );

// 4. Build the Library
        AprilTagLibrary result = new AprilTagLibrary.Builder()
            .addCluster(frontCluster)
            .addCluster(backCluster)
            .addCluster(redCluster)
            .addCluster(blueCluster)
            .build();

        return result;
    }

    public AprilTagLibrary createTagLibrary_WallClusterEachCorner() {

// front Wall Members (Cluster X offsets)
        AprilTagClusterMemberMetadata audLeftMemberA  = new AprilTagClusterMemberMetadata(30, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audLeftMemberB  = new AprilTagClusterMemberMetadata(31, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audRightMemberA = new AprilTagClusterMemberMetadata(32, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata audRightMemberB = new AprilTagClusterMemberMetadata(33, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);

// Back Wall Members (Cluster X offsets)
        AprilTagClusterMemberMetadata backLeftMemberA  = new AprilTagClusterMemberMetadata(34, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backLeftMemberB  = new AprilTagClusterMemberMetadata(35, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backRightMemberA = new AprilTagClusterMemberMetadata(36, new VectorF(-0.25f, 0.0f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata backRightMemberB = new AprilTagClusterMemberMetadata(37, new VectorF( 0.25f, 0.0f, 0.0f), tag325SizeInches);

// Red Wall Members (Cluster Y offsets)
        AprilTagClusterMemberMetadata redLeftMemberA  = new AprilTagClusterMemberMetadata(38, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redLeftMemberB  = new AprilTagClusterMemberMetadata(39, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redRightMemberA = new AprilTagClusterMemberMetadata(40, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata redRightMemberB = new AprilTagClusterMemberMetadata(41, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);

// Blue Wall Members (Cluster Y offsets)
        AprilTagClusterMemberMetadata blueLeftMemberA  = new AprilTagClusterMemberMetadata(42, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueLeftMemberB  = new AprilTagClusterMemberMetadata(43, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueRightMemberA = new AprilTagClusterMemberMetadata(44, new VectorF(0.0f, -0.25f, 0.0f), tag325SizeInches);
        AprilTagClusterMemberMetadata blueRightMemberB = new AprilTagClusterMemberMetadata(45, new VectorF(0.0f,  0.25f, 0.0f), tag325SizeInches);

// 3. Instantiate AprilTagClusterMetadata groups
// Order: members, name, shortName, fieldPosition, distanceUnit, fieldOrientation
        AprilTagClusterMetadata frontLeftCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(audLeftMemberA, audLeftMemberB)),
            "frontLeft", "AudLeft", new VectorF(-48.0f, 72.0f, 12.0f), DistanceUnit.INCH, facingMinusY
        );

        AprilTagClusterMetadata frontRightCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(audRightMemberA, audRightMemberB)),
            "frontRight", "AudRight", new VectorF(48.0f, 72.0f, 12.0f), DistanceUnit.INCH, facingMinusY
        );

        AprilTagClusterMetadata backLeftCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(backLeftMemberA, backLeftMemberB)),
            "BackLeft", "BackLeft", new VectorF(-48.0f, -72.0f, 12.0f), DistanceUnit.INCH, facingPlusY
        );

        AprilTagClusterMetadata backRightCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(backRightMemberA, backRightMemberB)),
            "BackRight", "BackRight", new VectorF(48.0f, -72.0f, 12.0f), DistanceUnit.INCH, facingPlusY
        );

        AprilTagClusterMetadata redLeftCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(redLeftMemberA, redLeftMemberB)),
            "RedLeft", "RedLeft", new VectorF(-72.0f, -48.0f, 12.0f), DistanceUnit.INCH, facingPlusX
        );

        AprilTagClusterMetadata redRightCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(redRightMemberA, redRightMemberB)),
            "RedRight", "RedRight", new VectorF(-72.0f, 48.0f, 12.0f), DistanceUnit.INCH, facingPlusX
        );

        AprilTagClusterMetadata blueLeftCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(blueLeftMemberA, blueLeftMemberB)),
            "BlueLeft", "BlueLeft", new VectorF(72.0f, -48.0f, 12.0f), DistanceUnit.INCH, facingMinusX
        );

        AprilTagClusterMetadata blueRightCluster = new AprilTagClusterMetadata(
            new ArrayList<>(Arrays.asList(blueRightMemberA, blueRightMemberB)),
            "BlueRight", "BlueRight", new VectorF(72.0f, 48.0f, 12.0f), DistanceUnit.INCH, facingMinusX
        );

        // 4. Add clusters to the library mapping builder
        AprilTagLibrary result = new AprilTagLibrary.Builder()
            .addCluster(frontLeftCluster)
            .addCluster(frontRightCluster)
            .addCluster(backLeftCluster)
            .addCluster(backRightCluster)
            .addCluster(redLeftCluster)
            .addCluster(redRightCluster)
            .addCluster(blueLeftCluster)
            .addCluster(blueRightCluster)
            .build();

        return result;
    }


    public AprilTagProcessor createAprilTagProcessor(AprilTagLibrary tags) {
        AprilTagProcessor result = new AprilTagProcessor.Builder()
            .setTagLibrary(tags)
            .build();
        return result;
    }
} // public class TestTagLibraries
