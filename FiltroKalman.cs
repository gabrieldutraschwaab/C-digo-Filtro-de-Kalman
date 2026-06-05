// ============================================================
// FiltroKalmanGeolocalizacao.cs - VERSÃO CORRIGIDA E ULTRA STABLE
// ============================================================

using UnityEngine;
using UnityEngine.XR;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

public class FiltroKalmanGeolocalizacao : MonoBehaviour
{
    private Vector<double> x_estado;
    private Matrix<double> P_covariancia;
    private Matrix<double> Q_ruido_processo;
    private Matrix<double> R_ruido_medicao;
    private Matrix<double> H_medicao;

    private Vector3 posicaoAnteriorOpenXR;
    private Quaternion orientacaoAnteriorOpenXR;
    private bool primeiroFrameVIO = true;

    [Header("Ruído do Processo (VIO/IMU)")]
    public float qPosicao = 0.1f;      
    public float qVelocidade = 0.2f;

    [Header("Ruído da Medição (UWB)")]
    public float rUWB = 0.5f;

    [Header("Teste Chi-Quadrado (NLOS)")]
    public double chiQuadradoThreshold = 9999999.0; 

    [Header("Restrições de Mapa (Map Matching)")]
    public bool usarMapMatching = false;
    public float raioMapMatching = 1.0f;

    private bool filtroInicializado = false;
    private System.Collections.Generic.List<InputDevice> dispositivos = new System.Collections.Generic.List<InputDevice>();
    private UnityEngine.AI.NavMeshHit hit;

    private static readonly MatrixBuilder<double> M = Matrix<double>.Build;
    private static readonly VectorBuilder<double> V = Vector<double>.Build;

    void Start()
    {
        InicializarFiltro();
        InicializarOpenXR();
    }

    void InicializarFiltro()
    {
        x_estado = V.Dense(6, 0.0);
        
        x_estado[0] = transform.position.x;
        x_estado[1] = transform.position.y;
        x_estado[2] = transform.position.z;

        P_covariancia = M.DenseIdentity(6) * 1.0; 
        Q_ruido_processo = M.Dense(6, 6, 0.0);
        AtualizarMatrizQ();
        
        R_ruido_medicao = M.DenseIdentity(3) * (rUWB * rUWB);
        H_medicao = M.Dense(3, 6, 0.0);
        H_medicao[0, 0] = 1.0;
        H_medicao[1, 1] = 1.0;
        H_medicao[2, 2] = 1.0;
        
        filtroInicializado = true;
    }

    void InicializarOpenXR()
    {
        InputDevices.GetDevicesAtXRNode(XRNode.Head, dispositivos);
        if (dispositivos.Count > 0)
        {
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos)) posicaoAnteriorOpenXR = pos;
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rot)) orientacaoAnteriorOpenXR = rot;
        }
    }

    void Update()
    {
        if (!filtroInicializado) return;

        float dt = Time.deltaTime;
        if (dt <= 0.0001f || dt > 0.5f) return; 

        Vector3 posicaoAtualOpenXR = Vector3.zero;
        Quaternion orientacaoAtualOpenXR = Quaternion.identity;
        bool dadosValidos = false;

        InputDevices.GetDevicesAtXRNode(XRNode.Head, dispositivos);
        if (dispositivos.Count > 0)
        {
            if (dispositivos[0].TryGetFeatureValue(CommonUsages.devicePosition, out posicaoAtualOpenXR) &&
                dispositivos[0].TryGetFeatureValue(CommonUsages.deviceRotation, out orientacaoAtualOpenXR))
            {
                dadosValidos = true;
            }
        }

        if (!dadosValidos)
        {
            if (GameObject.Find("Operador_Alvo") != null) {
                posicaoAtualOpenXR = GameObject.Find("Operador_Alvo").transform.position;
            } else {
                posicaoAtualOpenXR = new Vector3((float)x_estado[0], (float)x_estado[1], (float)x_estado[2]);
            }
            orientacaoAtualOpenXR = Quaternion.identity;
        }

        if (primeiroFrameVIO)
        {
            posicaoAnteriorOpenXR = posicaoAtualOpenXR;
            orientacaoAnteriorOpenXR = orientacaoAtualOpenXR;
            primeiroFrameVIO = false;
            return;
        }

        Vector3 deltaPosicao = posicaoAtualOpenXR - posicaoAnteriorOpenXR;
        Vector3 velocidadeInstantanea = deltaPosicao / dt;

        if(velocidadeInstantanea.magnitude > 50f) velocidadeInstantanea = Vector3.zero;

        var F_t = M.DenseIdentity(6);
        F_t[0, 3] = dt;
        F_t[1, 4] = dt;
        F_t[2, 5] = dt;

        var u = V.Dense(6, 0.0);
        u[0] = deltaPosicao.x;
        u[1] = deltaPosicao.y;
        u[2] = deltaPosicao.z;

        AtualizarMatrizQ();

        x_estado = (F_t * x_estado) + u;
        P_covariancia = (F_t * P_covariancia * F_t.Transpose()) + Q_ruido_processo;

        if (!double.IsNaN(x_estado[0]) && !double.IsNaN(x_estado[1]) && !double.IsNaN(x_estado[2]))
        {
            transform.position = new Vector3((float)x_estado[0], (float)x_estado[1], (float)x_estado[2]);
            transform.rotation = orientacaoAtualOpenXR;
        }

        posicaoAnteriorOpenXR = posicaoAtualOpenXR;
        orientacaoAnteriorOpenXR = orientacaoAtualOpenXR;
    }

    public void ReceberMedicaoUWB(Vector3 posicaoUWB)
    {
        if (!filtroInicializado) return;
        if (float.IsNaN(posicaoUWB.x) || float.IsNaN(posicaoUWB.y) || float.IsNaN(posicaoUWB.z)) return;

        var z_medicao = V.Dense(3);
        z_medicao[0] = posicaoUWB.x;
        z_medicao[1] = posicaoUWB.y;
        z_medicao[2] = posicaoUWB.z;

        var y_inovacao = z_medicao - (H_medicao * x_estado);
        R_ruido_medicao = M.DenseIdentity(3) * (rUWB * rUWB);
        
        var S_cov_inovacao = (H_medicao * P_covariancia * H_medicao.Transpose()) + R_ruido_medicao;6
        
        double determinante = S_cov_inovacao.Determinant();
        if (double.IsNaN(determinante) || System.Math.Abs(determinante) < 1e-6) return;

        var S_inv = S_cov_inovacao.Inverse();
        double distMahalanobis = y_inovacao * S_inv * y_inovacao;

        if (distMahalanobis > chiQuadradoThreshold || double.IsNaN(distMahalanobis)) return;

        var K_ganho = P_covariancia * H_medicao.Transpose() * S_inv;
        
        var novo_estado = x_estado + (K_ganho * y_inovacao);
        var nova_covariancia = (M.DenseIdentity(6) - (K_ganho * H_medicao)) * P_covariancia;

        if (!double.IsNaN(novo_estado[0]) && !double.IsNaN(nova_covariancia[0, 0]))
        {
            x_estado = novo_estado;
            P_covariancia = nova_covariancia;
        }
    }

    void AtualizarMatrizQ()
    {
        Q_ruido_processo[0, 0] = qPosicao * qPosicao;
        Q_ruido_processo[1, 1] = qPosicao * qPosicao;
        Q_ruido_processo[2, 2] = qPosicao * qPosicao;
        Q_ruido_processo[3, 3] = qVelocidade * qVelocidade;
        Q_ruido_processo[4, 4] = qVelocidade * qVelocidade;
        Q_ruido_processo[5, 5] = qVelocidade * qVelocidade;
    }
}
