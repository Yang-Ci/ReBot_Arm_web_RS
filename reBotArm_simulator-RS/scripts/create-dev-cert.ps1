param(
  [string]$HostIp = "",
  [string]$OutDir = ".certs",
  [string]$Password = "rebotarm-dev"
)

$ErrorActionPreference = "Stop"

if (-not $HostIp) {
  $HostIp = [System.Net.NetworkInformation.NetworkInterface]::GetAllNetworkInterfaces() |
    Where-Object { $_.OperationalStatus -eq [System.Net.NetworkInformation.OperationalStatus]::Up } |
    ForEach-Object { $_.GetIPProperties().UnicastAddresses } |
    Where-Object { $_.Address.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork -and $_.Address.ToString() -notlike "127.*" } |
    Select-Object -ExpandProperty Address -First 1 |
    ForEach-Object { $_.ToString() }
}

if (-not $HostIp) {
  throw "No LAN IPv4 address found. Pass one explicitly, for example: scripts/create-dev-cert.ps1 -HostIp 192.168.1.23"
}

$ResolvedOutDir = Join-Path (Resolve-Path ".") $OutDir
New-Item -ItemType Directory -Force -Path $ResolvedOutDir | Out-Null

Add-Type -AssemblyName System.Security
Add-Type -AssemblyName System.Net.Primitives

$Now = [DateTimeOffset]::UtcNow.AddMinutes(-5)
$RootUntil = $Now.AddYears(5)
$ServerUntil = $Now.AddYears(2)

$RootRsa = [System.Security.Cryptography.RSA]::Create(2048)
$RootRequest = [System.Security.Cryptography.X509Certificates.CertificateRequest]::new(
  "CN=reBot Arm Local Dev Root",
  $RootRsa,
  [System.Security.Cryptography.HashAlgorithmName]::SHA256,
  [System.Security.Cryptography.RSASignaturePadding]::Pkcs1
)

$RootRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509BasicConstraintsExtension]::new($true, $true, 1, $true)
)
$RootRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509KeyUsageExtension]::new(
    [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::KeyCertSign -bor
    [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::CrlSign -bor
    [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::DigitalSignature,
    $true
  )
)
$RootRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509SubjectKeyIdentifierExtension]::new($RootRequest.PublicKey, $false)
)

$RootCert = $RootRequest.CreateSelfSigned($Now, $RootUntil)

$ServerRsa = [System.Security.Cryptography.RSA]::Create(2048)
$ServerRequest = [System.Security.Cryptography.X509Certificates.CertificateRequest]::new(
  "CN=$HostIp",
  $ServerRsa,
  [System.Security.Cryptography.HashAlgorithmName]::SHA256,
  [System.Security.Cryptography.RSASignaturePadding]::Pkcs1
)

$SanBuilder = [System.Security.Cryptography.X509Certificates.SubjectAlternativeNameBuilder]::new()
$SanBuilder.AddDnsName("localhost")
$SanBuilder.AddDnsName("rebotarm.local")
$SanBuilder.AddIpAddress([System.Net.IPAddress]::Parse("127.0.0.1"))
$SanBuilder.AddIpAddress([System.Net.IPAddress]::Parse($HostIp))

$ServerRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509BasicConstraintsExtension]::new($false, $false, 0, $true)
)
$ServerRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509KeyUsageExtension]::new(
    [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::DigitalSignature -bor
    [System.Security.Cryptography.X509Certificates.X509KeyUsageFlags]::KeyEncipherment,
    $true
  )
)

$ServerAuthOids = [System.Security.Cryptography.OidCollection]::new()
$ServerAuthOids.Add([System.Security.Cryptography.Oid]::new("1.3.6.1.5.5.7.3.1")) | Out-Null
$ServerRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509EnhancedKeyUsageExtension]::new($ServerAuthOids, $false)
)
$ServerRequest.CertificateExtensions.Add($SanBuilder.Build($false))
$ServerRequest.CertificateExtensions.Add(
  [System.Security.Cryptography.X509Certificates.X509SubjectKeyIdentifierExtension]::new($ServerRequest.PublicKey, $false)
)

$Serial = New-Object byte[] 16
$Rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
$Rng.GetBytes($Serial)
$Rng.Dispose()
$ServerCert = $ServerRequest.Create($RootCert, $Now, $ServerUntil, $Serial)
$ServerCertWithKey = [System.Security.Cryptography.X509Certificates.RSACertificateExtensions]::CopyWithPrivateKey($ServerCert, $ServerRsa)

$RootCer = Join-Path $ResolvedOutDir "rebotarm-local-root-ca.cer"
$ServerCrt = Join-Path $ResolvedOutDir "rebotarm-local-server.crt"
$ServerKey = Join-Path $ResolvedOutDir "rebotarm-local-server.key"

function Write-PemFile {
  param(
    [string]$Path,
    [string]$Label,
    [byte[]]$DerBytes
  )

  $Base64 = [Convert]::ToBase64String($DerBytes)
  $Lines = for ($i = 0; $i -lt $Base64.Length; $i += 64) {
    $Base64.Substring($i, [Math]::Min(64, $Base64.Length - $i))
  }

  $Pem = @("-----BEGIN $Label-----") + $Lines + @("-----END $Label-----", "")
  [System.IO.File]::WriteAllLines($Path, $Pem)
}

[System.IO.File]::WriteAllBytes($RootCer, $RootCert.Export([System.Security.Cryptography.X509Certificates.X509ContentType]::Cert))
Write-PemFile -Path $ServerCrt -Label "CERTIFICATE" -DerBytes $ServerCert.Export([System.Security.Cryptography.X509Certificates.X509ContentType]::Cert)
Write-PemFile -Path $ServerKey -Label "PRIVATE KEY" -DerBytes $ServerRsa.ExportPkcs8PrivateKey()

$RootCert.Dispose()
$ServerCert.Dispose()
$ServerCertWithKey.Dispose()
$RootRsa.Dispose()
$ServerRsa.Dispose()

Write-Host ""
Write-Host "Created HTTPS development certificate:"
Write-Host "  Server CRT: $ServerCrt"
Write-Host "  Server KEY: $ServerKey"
Write-Host "  Root CA:    $RootCer"
Write-Host ""
Write-Host "Start HTTPS:"
Write-Host "  npm run start:https"
Write-Host ""
Write-Host "Android install step:"
Write-Host "  Copy rebotarm-local-root-ca.cer to the phone and install it as a trusted CA certificate."
Write-Host "  Only do this for a certificate you generated yourself on this computer."
Write-Host "  Then open: https://$HostIp`:3444"
