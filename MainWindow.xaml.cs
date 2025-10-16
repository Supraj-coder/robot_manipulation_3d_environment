using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using HelixToolkit.Wpf;

namespace Project
{
    public partial class MainWindow : Window
    {
        Model3DGroup robot;
        Model3DGroup Base;
        Model3DGroup link1;
        Model3DGroup link2;
        Model3DGroup link3;
        Model3DGroup end;
        Model3DGroup Object;
        Model3D cube;

        ModelImporter importer;

        Point3D joint1_p;
        Point3D joint2_p;
        Point3D joint3_p;

        Double link1_len;
        Double link2_len;
        Double link3_len;
        Double link0_len;

        Vector3D ee_global;
        Vector3D ee_global_x;
        Vector3D ee_global_y;
        Vector3D ee_global_z;

        Vector3D cube_global;
        Vector3D cube_global_x;
        Vector3D cube_global_y;
        Vector3D cube_global_z;

        Vector3D ee_orientation_u;
        Vector3D ee_orientation_v;
        Vector3D ee_orientation_w;

        Vector3D cube_orientation_u;
        Vector3D cube_orientation_v;
        Vector3D cube_orientation_w;

        Point3D tcp;
        Boolean cube_picked;

        Point3D joint1_ref;
        Point3D joint2_ref;
        Point3D joint3_ref;
        Point3D tcp_ref;

        public MainWindow()
        {
            InitializeComponent();
            robot = new Model3DGroup();
            importer = new ModelImporter();
            Base = importer.Load(@"asset/Base.obj");
            link1 = importer.Load(@"asset/link1.obj");
            link2 = importer.Load(@"asset/link2.obj");
            link3 = importer.Load(@"asset/link3.obj");
            end = importer.Load(@"asset/end.obj");
            robot.Children.Add(Base);
            Base.Children.Add(link1);
            link1.Children.Add(link2);
            link2.Children.Add(link3);
            link3.Children.Add(end);
            model1.Content = robot;

            joint1_p = new Point3D(0, 0, 3.5);
            joint2_p = new Point3D(0, 6, 3.5);
            joint3_p = new Point3D(0, 10, 3.5);
            tcp = new Point3D(0, 13.75, 3.5);

            joint1_ref = new Point3D();
            joint2_ref = new Point3D();
            joint3_ref = new Point3D();
            tcp_ref = new Point3D();
            joint1_ref = joint1_p;
            joint2_ref = joint2_p;
            joint3_ref = joint3_p;
            tcp_ref = tcp;

            link0_len = Math.Sqrt(Math.Pow(joint1_p.X, 2) + Math.Pow(joint1_p.Y, 2) + Math.Pow(joint1_p.Z, 2));
            link1_len = Math.Sqrt(Math.Pow((joint1_p.X - joint2_p.X), 2) + Math.Pow((joint1_p.Y - joint2_p.Y), 2) + Math.Pow((joint1_p.Z - joint2_p.Z), 2));
            link2_len = Math.Sqrt(Math.Pow((joint2_p.X - joint3_p.X), 2) + Math.Pow((joint2_p.Y - joint3_p.Y), 2) + Math.Pow((joint2_p.Z - joint3_p.Z), 2));
            link3_len = Math.Sqrt(Math.Pow((joint3_p.X - tcp.X), 2) + Math.Pow((joint3_p.Y - tcp.Y), 2) + Math.Pow((joint3_p.Z - tcp.Z), 2));

            joint1_show.Text = $"Joint 1: {joint1.Value.ToString("F3")}";
            joint2_show.Text = $"Joint 2: {joint2.Value.ToString("F3")}";
            joint3_show.Text = $"Joint 3: {joint3.Value.ToString("F3")}";
            base_show.Text = $"Base : {base_slider.Value.ToString("F3")}";

            px_e.Text = $"{tcp.X.ToString("F2")}";
            py_e.Text = $"{tcp.Y.ToString("F2")}";
            pz_e.Text = $"{tcp.Z.ToString("F2")}";

            cube_picked = false;
            pick.IsEnabled = false;
            drop.IsEnabled = false;

            ee_orientation_u.X = 1.0;
            ee_orientation_u.Y = 0.0;
            ee_orientation_u.Z = 0.0;

            ee_orientation_v.X = 0.0;
            ee_orientation_v.Y = 1.0;
            ee_orientation_v.Z = 0.0;

            ee_orientation_w.X = 0.0;
            ee_orientation_w.Y = 0.0;
            ee_orientation_w.Z = 1.0;

            ux_e.Text = $"{ee_orientation_u.X.ToString("F2")}";
            uy_e.Text = $"{ee_orientation_u.Y.ToString("F2")}";
            uz_e.Text = $"{ee_orientation_u.Z.ToString("F2")}";

            vx_e.Text = $"{ee_orientation_v.X.ToString("F2")}";
            vy_e.Text = $"{ee_orientation_v.Y.ToString("F2")}";
            vz_e.Text = $"{ee_orientation_v.Z.ToString("F2")}";

            wx_e.Text = $"{ee_orientation_w.X.ToString("F2")}";
            wy_e.Text = $"{ee_orientation_w.Y.ToString("F2")}";
            wz_e.Text = $"{ee_orientation_w.Z.ToString("F2")}";

        }

        private void object_sync()
        {
            if (cube_picked)
            {
                cube_x.Value = -tcp.X;
                cube_y.Value = tcp.Y+0.75;
                cube_z.Value = tcp.Z-0.5;
            }
        }

        private void end_effector_data_update()
        {
            //Position Update
            Double j1 = (Math.PI / 180) * joint1.Value;
            Double j2 = (Math.PI / 180) * joint2.Value;
            Double j3 = (Math.PI / 180) * joint3.Value;
            Double b = (Math.PI / 180) * base_slider.Value;
            Double local_y = link1_len * Math.Cos(j1) + link2_len * Math.Cos(j1+j2) + link3_len * Math.Cos(j1+j2+j3);
            Double local_x = link1_len * Math.Sin(j1) + link2_len * Math.Sin(j1+j2) + link3_len * Math.Sin(j1+j2+j3);
            //Double r = Math.Sqrt((local_x * local_x) + (local_y * local_y));
            ee_global.Z = link0_len + local_x;
            ee_global.X = local_y * Math.Sin(b);
            ee_global.Y = local_y * Math.Cos(b);
            tcp.X = ee_global.X;
            tcp.Y = ee_global.Y;
            tcp.Z = ee_global.Z;

            ee_global_x = new Vector3D(tcp.X, 0, 0);
            ee_global_y = new Vector3D(0, tcp.Y, 0);
            ee_global_z = new Vector3D(0, 0, tcp.Z);

            joint3_data_update();
            joint2_data_update();
            ee_orientation_calc();
        }

        private void joint3_data_update()
        {
            Double j1 = (Math.PI / 180) * joint1.Value;
            Double j2 = (Math.PI / 180) * joint2.Value;
            Double b = (Math.PI / 180) * base_slider.Value;
            Double local_y = link1_len * Math.Cos(j1) + link2_len * Math.Cos(j1 + j2);
            Double local_x = link1_len * Math.Sin(j1) + link2_len * Math.Sin(j1 + j2);
            joint3_ref.Z = link0_len + local_x;
            joint3_ref.X = local_y * Math.Sin(b);
            joint3_ref.Y = local_y * Math.Cos(b);
        }

        private void joint2_data_update()
        {
            Double j1 = (Math.PI / 180) * joint1.Value;
            Double b = (Math.PI / 180) * base_slider.Value;
            Double local_y = link1_len * Math.Cos(j1);
            Double local_x = link1_len * Math.Sin(j1);
            joint2_ref.Z = link0_len + local_x;
            joint2_ref.X = local_y * Math.Sin(b);
            joint2_ref.Y = local_y * Math.Cos(b);
        }

        private void end_effector_data_display()
        {
            px_e.Text = $"{ee_global.X.ToString("F2")}";
            py_e.Text = $"{ee_global.Y.ToString("F2")}";
            pz_e.Text = $"{ee_global.Z.ToString("F2")}";

            ux_e.Text = $"{ee_orientation_u.X.ToString("F2")}";
            uy_e.Text = $"{ee_orientation_u.Y.ToString("F2")}";
            uz_e.Text = $"{ee_orientation_u.Z.ToString("F2")}";

            vx_e.Text = $"{ee_orientation_v.X.ToString("F2")}";
            vy_e.Text = $"{ee_orientation_v.Y.ToString("F2")}";
            vz_e.Text = $"{ee_orientation_v.Z.ToString("F2")}";

            wx_e.Text = $"{ee_orientation_w.X.ToString("F2")}";
            wy_e.Text = $"{ee_orientation_w.Y.ToString("F2")}";
            wz_e.Text = $"{ee_orientation_w.Z.ToString("F2")}";

        }

        private void move_base(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), base_slider.Value));
            Base.Transform = rotateTransform3D;
            base_show.Text = $"Base : {base_slider.Value.ToString("F3")}";            

            end_effector_data_update();
            end_effector_data_display();
            object_sync();
        }

        private void move_joint3(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint3.Value));
            rotateTransform3D.CenterX = joint3_p.X;
            rotateTransform3D.CenterY = joint3_p.Y;
            rotateTransform3D.CenterZ = joint3_p.Z;
            end.Transform = rotateTransform3D;
            joint3_show.Text = $"Joint 3: {joint3.Value.ToString("F3")}";

            end_effector_data_update();
            end_effector_data_display();
            object_sync();
        }

        private void move_joint2(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint2.Value));
            rotateTransform3D.CenterX = joint2_p.X;
            rotateTransform3D.CenterY = joint2_p.Y;
            rotateTransform3D.CenterZ = joint2_p.Z;
            link3.Transform = rotateTransform3D;
            joint2_show.Text = $"Joint 2: {joint2.Value.ToString("F3")}";            

            end_effector_data_update();
            end_effector_data_display();
            object_sync();
        }

        private void ee_orientation_calc()
        {
            Matrix3D m = new Matrix3D();
            m.M11 = 1; m.M12 = 0; m.M13 = 0; m.M14 = 0;
            m.M21 = 0; m.M22 = 1; m.M23 = 0; m.M24 = 0;
            m.M31 = 0; m.M32 = 0; m.M33 = 1; m.M34 = 0;
            m.OffsetX = 0; m.OffsetY = 0; m.OffsetZ = 0; m.M44 = 1;
            Matrix3D z_rot = new Matrix3D();
            z_rot.M11 = Math.Cos((Math.PI / 180) * base_slider.Value); z_rot.M12 = -Math.Sin((Math.PI / 180) * base_slider.Value); z_rot.M13 = 0; z_rot.M14 = 0;
            z_rot.M21 = Math.Sin((Math.PI / 180) * base_slider.Value); z_rot.M22 = Math.Cos((Math.PI / 180) * base_slider.Value); z_rot.M23 = 0; z_rot.M24 = 0;
            z_rot.M31 = 0; z_rot.M32 = 0; z_rot.M33 = 1; z_rot.M34 = 0;
            z_rot.OffsetX = 0; z_rot.OffsetY = 0; z_rot.OffsetZ = 0; z_rot.M44 = 1;
            Matrix3D x_rot1 = new Matrix3D();
            x_rot1.M11 = 1; x_rot1.M12 = 0; x_rot1.M13 = 0; x_rot1.M14 = joint2_ref.X;
            x_rot1.M21 = 0; x_rot1.M22 = Math.Cos((Math.PI / 180) * joint1.Value); x_rot1.M23 = -Math.Sin((Math.PI / 180) * joint1.Value); x_rot1.M24 = joint2_ref.Y;
            x_rot1.M31 = 0; x_rot1.M32 = Math.Sin((Math.PI / 180) * joint1.Value); x_rot1.M33 = Math.Cos((Math.PI / 180) * joint1.Value); x_rot1.M34 = joint2_ref.Z;
            x_rot1.OffsetX = 0; x_rot1.OffsetY = 0; x_rot1.OffsetZ = 0; x_rot1.M44 = 1;
            Matrix3D x_rot2 = new Matrix3D();
            x_rot2.M11 = 1; x_rot2.M12 = 0; x_rot2.M13 = 0; x_rot2.M14 = joint3_ref.X;
            x_rot2.M21 = 0; x_rot2.M22 = Math.Cos((Math.PI / 180) * (joint1.Value + joint2.Value)); x_rot2.M23 = -Math.Sin((Math.PI / 180) * (joint1.Value + joint2.Value)); x_rot2.M24 = joint3_ref.Y;
            x_rot2.M31 = 0; x_rot2.M32 = Math.Sin((Math.PI / 180) * (joint1.Value + joint2.Value)); x_rot2.M33 = Math.Cos((Math.PI / 180) * (joint1.Value + joint2.Value)); x_rot2.M34 = joint3_ref.Z;
            x_rot2.OffsetX = 0; x_rot2.OffsetY = 0; x_rot2.OffsetZ = 0; x_rot2.M44 = 1;
            Matrix3D x_rot3 = new Matrix3D();
            x_rot3.M11 = 1; x_rot3.M12 = 0; x_rot3.M13 = 0; x_rot3.M14 = tcp.X;
            x_rot3.M21 = 0; x_rot3.M22 = Math.Cos((Math.PI / 180) * (joint1.Value + joint2.Value + joint3.Value)); x_rot3.M23 = -Math.Sin((Math.PI / 180) * (joint1.Value + joint2.Value + joint3.Value)); x_rot3.M24 = tcp.Y;
            x_rot3.M31 = 0; x_rot3.M32 = Math.Sin((Math.PI / 180) * (joint1.Value + joint2.Value + joint3.Value)); x_rot3.M33 = Math.Cos((Math.PI / 180) * (joint1.Value + joint2.Value + joint3.Value)); x_rot3.M34 = tcp.Z;
            x_rot3.OffsetX = 0; x_rot3.OffsetY = 0; x_rot3.OffsetZ = 0; x_rot3.M44 = 1;
            Matrix3D trans1 = new Matrix3D();
            trans1.M11 = 1; trans1.M12 = 0; trans1.M13 = 0; trans1.M14 = 0;
            trans1.M21 = 0; trans1.M22 = 1; trans1.M23 = 0; trans1.M24 = 0;
            trans1.M31 = 0; trans1.M32 = 0; trans1.M33 = 1; trans1.M34 = link0_len;
            trans1.OffsetX = 0; trans1.OffsetY = 0; trans1.OffsetZ = 0; trans1.M44 = 1;
            Matrix3D trans2 = new Matrix3D();
            trans2.M11 = 1; trans2.M12 = 0; trans2.M13 = 0; trans2.M14 = 0;
            trans2.M21 = 0; trans2.M22 = 1; trans2.M23 = 0; trans2.M24 = 0;
            trans2.M31 = 0; trans2.M32 = 0; trans2.M33 = 1; trans2.M34 = link1_len;
            trans2.OffsetX = 0; trans2.OffsetY = 0; trans2.OffsetZ = 0; trans2.M44 = 1;
            Matrix3D trans3 = new Matrix3D();
            trans3.M11 = 1; trans3.M12 = 0; trans3.M13 = 0; trans3.M14 = 0;
            trans3.M21 = 0; trans3.M22 = 1; trans3.M23 = 0; trans3.M24 = 0;
            trans3.M31 = 0; trans3.M32 = 0; trans3.M33 = 1; trans3.M34 = link2_len;
            trans3.OffsetX = 0; trans3.OffsetY = 0; trans3.OffsetZ = 0; trans3.M44 = 1;
            Matrix3D trans4 = new Matrix3D();
            trans4.M11 = 1; trans4.M12 = 0; trans4.M13 = 0; trans4.M14 = 0;
            trans4.M21 = 0; trans4.M22 = 1; trans4.M23 = 0; trans4.M24 = 0;
            trans4.M31 = 0; trans4.M32 = 0; trans4.M33 = 1; trans4.M34 = link3_len;
            trans4.OffsetX = 0; trans4.OffsetY = 0; trans4.OffsetZ = 0; trans4.M44 = 1;

            m = z_rot * trans1 * x_rot1 * trans2 * x_rot2 * trans3 * x_rot3 * trans4;

            ee_orientation_u.X = m.M11;
            ee_orientation_u.Y = m.M21;
            ee_orientation_u.Z = m.M31;

            ee_orientation_v.X = m.M21;
            ee_orientation_v.Y = m.M22;
            ee_orientation_v.Z = m.M23;

            ee_orientation_w.X = m.M31;
            ee_orientation_w.Y = m.M32;
            ee_orientation_w.Z = m.M33;
        }

        private void move_joint1(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            RotateTransform3D rotateTransform3D = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), joint1.Value));
            rotateTransform3D.CenterX = joint1_p.X;
            rotateTransform3D.CenterY = joint1_p.Y;
            rotateTransform3D.CenterZ = joint1_p.Z;
            link2.Transform = rotateTransform3D;
            joint1_show.Text = $"Joint 1: {joint1.Value.ToString("F3")}";            

            end_effector_data_update();
            end_effector_data_display();
            object_sync();
        }

        private void cube_data_update()
        {
            //Position Update
            cube_global.X = cube_x.Value;
            cube_global.Y = cube_y.Value;
            cube_global.Z = cube_z.Value;

            cube_global_x = new Vector3D(cube_global.X, 0, 0);
            cube_global_y = new Vector3D(0, cube_global.Y, 0);
            cube_global_z = new Vector3D(0, 0, cube_global.Z);

            //Orientation update
            cube_orientation_u.X = 1.0;
            cube_orientation_u.Y = 0.0;
            cube_orientation_u.Z = 0.0;

            cube_orientation_v.X = 0.0;
            cube_orientation_v.Y = 1.0;
            cube_orientation_v.Z = 0.0;

            cube_orientation_w.X = 0.0;
            cube_orientation_w.Y = 0.0;
            cube_orientation_w.Z = 1.0;

            ux_c.Text = $"{cube_orientation_u.X.ToString("F2")}";
            uy_c.Text = $"{cube_orientation_u.Y.ToString("F2")}";
            uz_c.Text = $"{cube_orientation_u.Z.ToString("F2")}";

            vx_c.Text = $"{cube_orientation_v.X.ToString("F2")}";
            vy_c.Text = $"{cube_orientation_v.Y.ToString("F2")}";
            vz_c.Text = $"{cube_orientation_v.Z.ToString("F2")}";

            wx_c.Text = $"{cube_orientation_w.X.ToString("F2")}";
            wy_c.Text = $"{cube_orientation_w.Y.ToString("F2")}";
            wz_c.Text = $"{cube_orientation_w.Z.ToString("F2")}";
        }

        private void cube_data_display()
        {
            px_c.Text = $"{cube_global.X.ToString("F2")}";
            py_c.Text = $"{cube_global.Y.ToString("F2")}";
            pz_c.Text = $"{cube_global.Z.ToString("F2")}";

            ux_c.Text = $"{cube_orientation_u.X.ToString("F2")}";
            uy_c.Text = $"{cube_orientation_u.Y.ToString("F2")}";
            uz_c.Text = $"{cube_orientation_u.Z.ToString("F2")}";

            vx_c.Text = $"{cube_orientation_v.X.ToString("F2")}";
            vy_c.Text = $"{cube_orientation_v.Y.ToString("F2")}";
            vz_c.Text = $"{cube_orientation_v.Z.ToString("F2")}";

            wx_c.Text = $"{cube_orientation_w.X.ToString("F2")}";
            wy_c.Text = $"{cube_orientation_w.Y.ToString("F2")}";
            wz_c.Text = $"{cube_orientation_w.Z.ToString("F2")}";
        }

        private void add_cube(object sender, RoutedEventArgs e)
        {
            Object = new Model3DGroup();
            cube = importer.Load(@"asset/cube.obj");
            Object.Children.Add(cube);
            model2.Content = Object;

            cube_add.IsEnabled = false;
            cube_remove.IsEnabled = true;
            cube_x.IsEnabled = true;
            cube_y.IsEnabled = true;
            cube_z.IsEnabled = true;
            pick.IsEnabled = true;
            cube_y.Value = 10;
            MessageBox.Show("Cube added to the workspace!!");
            

            px_c.Text = $"{cube_x.Value.ToString("F2")}";
            py_c.Text = $"{cube_y.Value.ToString("F2")}";
            pz_c.Text = $"{cube_z.Value.ToString("F2")}";
        }

        private void remove_cube(object sender, RoutedEventArgs e)
        {
            if (!cube_picked)
            {
                model2.Content = null;
                cube_add.IsEnabled = true;
                cube_remove.IsEnabled = false;
                cube_y.Value = 0;
                cube_x.Value = 0;
                cube_z.Value = 0;
                cube_x.IsEnabled = false;
                cube_y.IsEnabled = false;
                cube_z.IsEnabled = false;
                MessageBox.Show("Cube removed from the workspace!!");
                

                ux_c.Text = "0";
                uy_c.Text = "0";
                uz_c.Text = "0";
                vx_c.Text = "0";
                vy_c.Text = "0";
                vz_c.Text = "0";
                wx_c.Text = "0";
                wy_c.Text = "0";
                wz_c.Text = "0";
                px_c.Text = "0";
                py_c.Text = "0";
                pz_c.Text = "0";
            }
            else
            {
                drop_cube(null, new RoutedEventArgs());
                remove_cube(null, new RoutedEventArgs());
            }

        }

        private void move_cube_x(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;            
            px_c.Text = $"{cube_x.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void move_cube_y(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;            
            py_c.Text = $"{cube_y.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void move_cube_z(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            TranslateTransform3D translateTransform3D = new TranslateTransform3D(new Vector3D(cube_x.Value, cube_y.Value, cube_z.Value));
            cube.Transform = translateTransform3D;            
            pz_c.Text = $"{cube_z.Value.ToString("F2")}";
            cube_data_update();
            cube_data_display();
        }

        private void pick_cube(object sender, RoutedEventArgs e)
        {
            if (!cube_picked)
            {
                cube_picked = true;
                pick.IsEnabled = false;
                drop.IsEnabled = true;
                object_sync();
                cube_x.IsEnabled = false;
                cube_y.IsEnabled = false;
                cube_z.IsEnabled = false;                
            }
        }

        private void drop_cube(object sender, RoutedEventArgs e)
        {
            if (cube_picked)
            {
                cube_picked = false;
                cube_z.Value = 0;
                pick.IsEnabled = true;
                drop.IsEnabled = false;
                cube_x.IsEnabled = true;
                cube_y.IsEnabled = true;
                cube_z.IsEnabled = true;
            }
        }

        private void reset_robot(object sender, RoutedEventArgs e)
        {
            joint1.Value = 0;
            joint2.Value = 0;
            joint3.Value = 0;
            base_slider.Value = 0;
        }

    }
}